# DEPENDENCIES AND IMPORTS
import sys as _sys
from pathlib import Path as _Path
# Look in the accompanying .deps folder for the supplied dependency packages.
_sys.path.insert(0, str(_Path(__file__).resolve().parent / '.deps'))
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
import json
import math
from pathlib import Path
import queue
import threading
import time
from urllib.parse import urlsplit
from PIL import Image


# Mission manages flight phases; ReportJobs exports reports in the background.
# Simulator performs the ZeroMQ remote API calls to CoppeliaSim.
# DT is the physics step size and TARGETS defines the inspection points.
from windbridge.mission import Mission
from windbridge.report_jobs import ReportJobs
from windbridge.simulator import ROOT, RESOURCE_ROOT, FROZEN, SCENE, Simulator
from windbridge.world import DT, TARGETS

# LOCALHOST SETTINGS
# Listen on this computer at port 8765 and serve the browser interface files.
HOST = '127.0.0.1'
PORT = 8765
URL = f'http://{HOST}:{PORT}'
WEB_ROOT = RESOURCE_ROOT / 'dashboard'


# INPUT VALIDATION
# Check that selected points are P1-P6, wind speed is 0-6 m/s, and wind
# direction is 0-360 degrees. Return consistent settings or reject the input.
def settings_from(data):
    """Reject malformed commands before they can enter the simulator thread."""
    if not isinstance(data, dict):
        raise ValueError('Settings must be an object.')
    selected = data.get('selected', list(range(1, 7)))
    if not isinstance(selected, list) or any(type(i) is not int or i not in range(1, 7) for i in selected):
        raise ValueError('Choose inspection points P1 to P6.')
    speed, angle = data.get('speed', 4), data.get('angle', 0)
    if type(speed) not in (int, float) or type(angle) not in (int, float):
        raise ValueError('Wind speed and direction must be numbers.')
    if not math.isfinite(speed) or not math.isfinite(angle) or not (0 <= speed <= 6 and 0 <= angle <= 360):
        raise ValueError('Use wind speed 0–6 m/s and direction 0–360°.')
    return {'selected': sorted(set(selected)), 'speed': float(speed), 'angle': float(angle)}


# DASHBOARD BACKEND
class Dashboard:
    # Prepare shared state, locks, command queues and the latest camera frame.
    # Create separate workers for simulator operations and image preparation.
    def __init__(self):
        # Protect shared data read by the HTTP, simulator and image worker threads.
        self.lock = threading.RLock()
        self.frames = threading.Condition(self.lock)
        # Queue operator commands in order; keep at most one image waiting to publish.
        self.commands = queue.Queue()
        self.images = queue.Queue(maxsize=1)
        self.closing = threading.Event()
        self.settings = settings_from({})
        # Keep a limited history for the live tracking-error graph.
        self.history = deque(maxlen=1500)
        self.jpeg = None
        self.frame_id = 0
        self.frame_times = deque(maxlen=200)
        self.frame_costs = deque(maxlen=200)
        self.frame_size = None
        self.started_wall = time.perf_counter()
        self.last_save = None
        self.reports = ReportJobs()
        # Initial values shown while the dashboard connects to the simulator.
        self.state = dict(ready=False, busy=False, running=False, paused=False,
                          finished=False, phase='connecting', time=0, target=0,
                          completed=[], dwell=0, tracking_error=0,
                          message='Connecting to CoppeliaSim…', error='')
        # Separate image preparation from the worker that owns simulation state.
        self.encoder = threading.Thread(target=self._encode, name='preview-jpeg', daemon=True)
        self.worker = threading.Thread(target=self._run, name='simulator-owner', daemon=True)

    def start(self):
        self.reports.restore(ROOT / 'results')
        self.encoder.start()
        self.worker.start()

    # Return the status, graph samples, settings, report state and recent video
    # frame rate that the browser needs to refresh its display.
    def snapshot(self):
        with self.lock:
            recent = [t for t in self.frame_times if t >= time.perf_counter() - 3]
            fps = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 else 0
            return {**self.state, 'settings': dict(self.settings), 'history': list(self.history),
                    'feed_fps': round(fps, 1), 'frame_id': self.frame_id,
                'frame_size': self.frame_size, 'last_save': self.last_save,
                'report': self.reports.snapshot(),
                'camera_read_ms': round(1000 * sum(self.frame_costs) / len(self.frame_costs), 1) if self.frame_costs else 0}

    # Check whether an operator action is allowed, then queue it for the
    # simulator worker. Browser request threads do not control the drone directly.
    def enqueue(self, action, data):
        settings = settings_from(data)
        if action == 'start' and not settings['selected']:
            raise ValueError('Select at least one inspection point before starting.')
        with self.lock:
            if not self.state['ready']:
                raise ValueError('CoppeliaSim is still connecting. Please wait.')
            if self.state['busy'] and action in ('start', 'reset'):
                raise ValueError('The previous action is still being processed.')
            if action == 'start' and self.state['running']:
                raise ValueError('An inspection is already running.')
            if action in ('start', 'reset'):
                self.state['busy'] = True
            self.state['error'] = ''
        self.commands.put((action, settings))

    # Update shared dashboard status while holding the lock so concurrent
    # browser requests do not read a partially updated status.
    def _set(self, **values):
        with self.lock:
            self.state.update(values)

    # Keep only the newest camera image. Replace an older waiting image when
    # necessary so the live feed does not accumulate delayed frames.
    def _offer_frame(self, image):
        try:
            self.images.put_nowait(image)
        except queue.Full:
            try:
                self.images.get_nowait()
            except queue.Empty:
                pass
            self.images.put_nowait(image)

    # Prepare JPEG images for the browser in a separate background worker.
    # Reuse simulator JPEG bytes when available, otherwise encode the image.
    def _encode(self):
        while not self.closing.is_set():
            try:
                image = self.images.get(timeout=.5)
            except queue.Empty:
                continue
            if isinstance(image, (bytes, bytearray)):
                jpeg = bytes(image)
                size = Image.open(BytesIO(jpeg)).size
            else:
                buffer = BytesIO()
                image.save(buffer, format='JPEG', quality=78)
                jpeg, size = buffer.getvalue(), image.size
            # Publish the newest JPEG and wake browser streams waiting for a new frame.
            with self.frames:
                self.jpeg = jpeg
                self.frame_id += 1
                self.frame_size = list(size)
                self.frame_times.append(time.perf_counter())
                self.frames.notify_all()

    # Request a camera preview through the simulator interface, queue the image
    # for the browser and measure how long the camera request took.
    def _preview(self, sim):
        start = time.perf_counter()
        self._offer_frame(sim.preview())
        self.frame_costs.append(time.perf_counter() - start)

    # Show selected inspection markers as pending and other markers as inactive
    # when preparing or resetting the scene.
    def _colors(self, sim):
        for target in TARGETS:
            sim.target_color(target.id, 'pending' if target.id in self.settings['selected'] else 'inactive')

    # Copy actual mission telemetry into browser status and tracking-error data.
    # The display uses fewer samples; saved telemetry keeps every physics step.
    def _publish(self, mission):
        row = mission.rows[-1] if mission.rows else None
        with self.lock:
            if row:
                # Decimate display data only; telemetry.csv retains every physics step.
                if not self.history or row['time'] - self.history[-1][0] >= .099:
                    self.history.append([round(row['time'], 3), round(row['tracking_error'], 5)])
            self.state.update(ready=True, busy=False, running=not mission.finished,
                              finished=mission.finished, paused=mission.paused,
                              phase=mission.phase, target=mission.target.id if mission.target else 0,
                              time=float(mission.state['time']), completed=sorted(mission.completed),
                              dwell=mission.dwell,
                              tracking_error=float(row['tracking_error']) if row else 0)

    # Save the mission measurements, events and routes, then remember the
    # results folder so the dashboard can identify the saved run.
    def _save(self, mission):
        folder = mission.save()
        with self.lock:
            self.last_save = str(folder.relative_to(ROOT))
        return folder

    # SIMULATOR AND MISSION LOOP
    # Own the CoppeliaSim connection, process queued actions and advance the
    # mission. Mission and Simulator modules handle planning and remote calls.
    def _run(self):
        mission = None
        sim = None
        saved = False
        next_step = time.perf_counter()
        next_frame = 0.
        next_state = 0.
        try:
            # Connect through Simulator, load the bridge scene and prepare its first view.
            sim = Simulator()
            self._colors(sim)
            self._preview(sim)
            self._set(ready=True, phase='ready', message='Choose points and wind, then start inspection.')
            while not self.closing.is_set():
                try:
                    # Merge adjacent slider changes; retain action ordering.
                    pending = []
                    while True:
                        try:
                            item = self.commands.get_nowait()
                            if item[0] == 'settings' and pending and pending[-1][0] == 'settings':
                                pending[-1] = item
                            else:
                                pending.append(item)
                        except queue.Empty:
                            break
                    for action, settings in pending:
                        self.settings = settings
                        # START: create a new wind-aware inspection mission and its results folder.
                        if action == 'start':
                            self.reports.clear()
                            if mission:
                                mission.close()
                                sim = Simulator()
                            self.history.clear()
                            self._set(message='Starting inspection…', phase='takeoff', error='')
                            name = 'localhost_' + datetime.now().strftime('%Y%m%d_%H%M%S')
                            mission = Mission(sim, 'wind', settings['selected'], settings['speed'],
                                              settings['angle'], name, preplan=False)
                            mission.run_dir = ROOT / 'results' / name
                            mission.run_dir.mkdir(parents=True, exist_ok=True)
                            saved = False
                            next_step = time.perf_counter()
                        # SETTINGS: apply changed wind or selected points to the active mission.
                        # Mission decides how to update the remaining route from the current state.
                        elif action == 'settings':
                            if mission and not mission.finished:
                                mission.apply(settings['selected'], settings['speed'], settings['angle'])
                            else:
                                self._colors(sim)
                                self._preview(sim)
                                self._set(message='Settings ready for the next inspection.')
                        # PAUSE/RESUME: toggle a controlled hold. Wind and physics still continue.
                        elif action == 'pause':
                            if mission and not mission.finished:
                                mission.toggle_hold()
                        # RESET: save an unfinished run as aborted, request its PDF and reload
                        # the scene into the ready state.
                        elif action == 'reset':
                            if mission:
                                if not mission.finished:
                                    mission.phase = 'aborted'
                                    mission.event('operator_reset')
                                folder = self._save(mission)
                                mission.close()
                                self.reports.request(folder)
                            mission = None
                            sim = Simulator()
                            self.history.clear()
                            self._colors(sim)
                            self._preview(sim)
                            self._set(busy=False, running=False, paused=False, finished=False,
                                      phase='ready', time=0, target=0, dwell=0, tracking_error=0,
                                      completed=[], error='', message='Ready for a new inspection.')
                    now = time.perf_counter()
                    if mission and not mission.finished:
                        # Advance one mission/control step when due. Mission.tick() computes the
                        # reference and calls the simulator interface to command and sample the drone.
                        if now >= next_step:
                            mission.tick()
                            next_step += DT
                            # Never accumulate catch-up work that creates a delayed view.
                            if next_step < time.perf_counter() - DT:
                                next_step = time.perf_counter()
                        now = time.perf_counter()
                        # Capture the live view on its own schedule, at most once per physics step.
                        if now >= next_frame:
                            self._preview(sim)
                            # Schedule from capture START. Adding the render time to
                            # every period previously halved the achievable feed rate.
                            next_frame = now + DT  # at most one frame per physics step
                        # Publish status and graph data approximately every 0.1 wall-clock seconds.
                        if now >= next_state:
                            self._publish(mission)
                            next_state = now + .1
                    # Save a finished run once and generate its report in a background job.
                    if mission and mission.finished and not saved:
                        self._preview(sim)
                        folder = self._save(mission)
                        self._publish(mission)
                        mission.close()
                        self._set(message=f'Inspection {mission.phase}. Results saved.', busy=False)
                        saved = True
                        self.reports.request(folder)
                # Expose mission-loop errors in the dashboard and mark an active run failed.
                except Exception as exc:
                    import traceback
                    traceback.print_exc()
                    print('Dashboard:', exc, flush=True)
                    message = str(exc)
                    if 'Resource temporarily unavailable' in message or 'current state' in message:
                        message = 'CoppeliaSim is waiting. Close any dialog in CoppeliaSim, then press Reset.'
                    self._set(error=message, busy=False)
                    if mission and not mission.finished:
                        mission.failure_reason = str(exc)
                        mission.phase = 'failed'
                        mission.finished = True
                # Small sleeps let HTTP + encoder work without busy-polling the simulator.
                self.closing.wait(max(.002, min(.02, next_step - time.perf_counter()))
                                  if mission and not mission.finished else .03)
        # Show an initial connection failure instead of reporting a ready simulator.
        except Exception as exc:
            self._set(error='Simulator connection: ' + str(exc), ready=False, phase='disconnected', busy=False)
            print(self.state['error'], flush=True)
        finally:
            # On shutdown, save any unfinished run as aborted and stop the simulation.
            if mission:
                try:
                    if not mission.finished:
                        mission.phase = 'aborted'
                        mission.event('dashboard_close')
                    self._save(mission)
                    mission.close()
                except Exception as exc:
                    print('Closing simulator:', exc, flush=True)
            elif sim:
                sim.stop()

    # Stop the background work, finish report jobs and save video-stream
    # statistics when the local dashboard is closed.
    def close(self):
        self.closing.set()
        with self.frames:
            self.frames.notify_all()
        self.worker.join(timeout=10)
        self.reports.close()
        report = {'frames_encoded': self.frame_id, 'resolution': self.frame_size,
                  'camera_read_mean_ms': round(1000 * sum(self.frame_costs) / len(self.frame_costs), 2) if self.frame_costs else None,
                  'preview_disk_writes': 0, 'stream': 'latest-frame MJPEG',
                  'physics_step_s': DT, 'source': 'CoppeliaSim OverviewCamera'}
        (ROOT / 'results' / 'localhost_stream.json').write_text(json.dumps(report, indent=2))


# LOCAL HTTP SERVER
# Allow browser requests and the video stream to run on separate threads.
class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


# BROWSER REQUEST HANDLERS
# Build HTTP handlers linked to this Dashboard instance. They serve files,
# return telemetry, stream video, download reports and accept commands.
def handler_for(app):
    # Handle one browser connection using HTTP/1.1.
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        # Handle the browser connection and tolerate disconnections during reloads.
        def handle(self):
            try:
                super().handle()
            except (ConnectionError, TimeoutError):
                # Reloading a browser naturally aborts its old video connection.
                pass

        # Reduce console noise by logging responses other than routine successes.
        def log_message(self, fmt, *args):
            if args and str(args[1] if len(args) > 1 else '') not in ('200', '202', '204'):
                print(fmt % args, flush=True)

        # Send the HTTP status, content type and response headers to the browser.
        def _headers(self, code, content_type, length=None):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            if length is not None:
                self.send_header('Content-Length', str(length))
            self.end_headers()

        # Convert Python status or error data into JSON and send it to the browser.
        def _json(self, code, value):
            body = json.dumps(value, allow_nan=False).encode()
            self._headers(code, 'application/json; charset=utf-8', len(body))
            self.wfile.write(body)

        # Accept requests only with this localhost dashboard Host and Origin,
        # preventing an unrelated webpage from submitting browser commands.
        def _local(self):
            # Loopback binding plus Host/Origin checks prevent drive-by commands.
            allowed = {f'127.0.0.1:{PORT}', f'localhost:{PORT}'}
            if self.headers.get('Host') not in allowed:
                self._json(403, {'error': 'Local dashboard requests only.'})
                return False
            origin = self.headers.get('Origin')
            if origin and origin not in {f'http://{h}' for h in allowed}:
                self._json(403, {'error': 'Local dashboard requests only.'})
                return False
            return True

        # Serve read requests: current status, application health, saved PDF,
        # camera frames, continuous video and the HTML/CSS/JavaScript interface.
        def do_GET(self):
            if not self._local():
                return
            path = urlsplit(self.path).path
            # STATUS: return mission phase, target, graph history, settings and feed FPS.
            if path == '/api/state':
                self._json(200, app.snapshot())
            # HEALTH: identify this application and report whether it is ready.
            elif path == '/api/health':
                self._json(200, {'application': 'WindBridge localhost', 'ready': app.snapshot()['ready'],
                                 'project_folder': str(ROOT), 'packaged': FROZEN,
                                 'scene_file': str(SCENE)})
            # PDF DOWNLOAD: send the latest generated report after the mission ends
            # or is reset; reject the request while a run or report is not ready.
            elif path == '/api/report':
                if app.snapshot()['running'] or app.snapshot()['busy']:
                    self._json(409, {'error': 'Finish or reset the current mission before downloading its report.'})
                    return
                try:
                    filename, body = app.reports.download()
                except (ValueError, OSError) as exc:
                    self._json(409, {'error': str(exc)})
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'application/pdf')
                self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            # SINGLE FRAME: return the newest available simulator JPEG.
            elif path == '/frame.jpg':
                with app.lock:
                    body = app.jpeg
                if body is None:
                    self._json(503, {'error': 'Camera is connecting.'})
                else:
                    self._headers(200, 'image/jpeg', len(body))
                    self.wfile.write(body)
            # LIVE VIDEO: stream successive JPEG frames over one HTTP connection.
            # Wait for new frames rather than sending a backlog of old images.
            elif path == '/stream.mjpg':
                self._headers(200, 'multipart/x-mixed-replace; boundary=frame')
                previous = -1
                try:
                    while not app.closing.is_set():
                        with app.frames:
                            app.frames.wait_for(lambda: app.frame_id != previous or app.closing.is_set(), timeout=2)
                            body, seq = app.jpeg, app.frame_id
                        if body is None:
                            continue
                        self.wfile.write((f'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(body)}\r\n'
                                          f'X-Frame-Id: {seq}\r\n\r\n').encode() + body + b'\r\n')
                        self.wfile.flush()
                        previous = seq
                except (ConnectionError, TimeoutError):
                    pass
                self.close_connection = True
            else:
                # INTERFACE FILES: serve only the known dashboard HTML, JavaScript and CSS.
                files = {'/': ('index.html', 'text/html; charset=utf-8'),
                         '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                         '/style.css': ('style.css', 'text/css; charset=utf-8')}
                if path not in files:
                    self._json(404, {'error': 'Not found.'})
                    return
                name, mime = files[path]
                body = (WEB_ROOT / name).read_bytes()
                self._headers(200, mime, len(body))
                self.wfile.write(body)

        # Read and validate browser commands for Start, wind/point changes,
        # Pause/Resume and Reset, then pass them to the command queue.
        def do_POST(self):
            if not self._local():
                return
            # Map the requested browser endpoint to a mission action.
            action = {'/api/start': 'start', '/api/settings': 'settings',
                      '/api/pause': 'pause', '/api/reset': 'reset'}.get(urlsplit(self.path).path)
            if not action:
                self._json(404, {'error': 'Unknown action.'})
                return
            try:
                # Limit request size and parse JSON before accepting the command.
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4096:
                    raise ValueError('Invalid request size.')
                data = json.loads(self.rfile.read(length))
                app.enqueue(action, data)
                self._json(202, {'accepted': True})
            except (ValueError, json.JSONDecodeError) as exc:
                self._json(400, {'error': str(exc)})
    return Handler


# START THE LOCALHOST DASHBOARD
# Start the HTTP server and workers, optionally open the browser and keep
# serving requests until shutdown. Then close the server and workers.
def launch(open_browser=True,stop_event=None):
    import webbrowser
    app = Dashboard()
    try:
        server = Server((HOST, PORT), handler_for(app))
    except OSError as exc:
        raise SystemExit(f'Cannot open {URL}. Close another WindBridge dashboard first. {exc}') from exc
    app.start()
    print(f'WindBridge dashboard: {URL}', flush=True)
    print('Keep this launcher and CoppeliaSim open. Ctrl+C stops the local dashboard.', flush=True)
    if open_browser:
        threading.Timer(1, lambda: webbrowser.open(URL)).start()
    if stop_event is not None:
        # Wait for a shutdown signal, then stop the HTTP server from another thread.
        def watch_stop():
            stop_event.wait()
            server.shutdown()
        threading.Thread(target=watch_stop,name='dashboard-shutdown',daemon=True).start()
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        app.close()


# START THE PROJECT FROM PYTHON
# Support the dependency self-check, verify the scene exists, reuse or
# launch CoppeliaSim and coordinate application shutdown.
def main():
    """Run this readable source version instead of the packaged EXE."""
    import signal
    from windbridge.presentation import ensure_simulator, self_check
    # Check imports and scene/interface files without starting a simulation.
    if '--self-check' in _sys.argv:
        return 0 if self_check() else 1
    if not SCENE.is_file():
        raise RuntimeError('Keep the scene and supporting source folders together.')
    # Prepare saved results and connect to this bridge scene or start CoppeliaSim.
    (ROOT / 'results').mkdir(exist_ok=True)
    process = ensure_simulator()
    stop = threading.Event()
    # Convert Ctrl+C or a termination signal into an orderly dashboard shutdown.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    if process is not None:
        # If this program launched CoppeliaSim, stop the dashboard when that
        # simulator process closes.
        def watch_simulator():
            process.wait()
            stop.set()
        threading.Thread(target=watch_simulator, daemon=True).start()
    try:
        # Start the dashboard; --no-browser leaves browser opening to the operator.
        launch(open_browser='--no-browser' not in _sys.argv, stop_event=stop)
    finally:
        stop.set()
        # Close the simulator process only when this source launcher started it.
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait(timeout=8)
    return 0


# PROGRAM ENTRY POINT
# Run main() when this file is executed, rather than imported as a module.
if __name__ == '__main__':
    raise SystemExit(main())
