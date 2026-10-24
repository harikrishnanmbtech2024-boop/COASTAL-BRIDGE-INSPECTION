-- Rigid-body quadrotor flight controller. Position and SO(3) attitude feedback.
-- Applied thrust, aerodynamic disturbance and torque drive CoppeliaSim dynamics.
sim=require('sim')
local H,ref,refv,refa,yaw,integ,last,trail,reftrail,arrows,lastArrow
local windSpeed,windAngle=4,0
local function dot(a,b) return a[1]*b[1]+a[2]*b[2]+a[3]*b[3] end
local function norm(a) return math.sqrt(dot(a,a)) end
local function cross(a,b) return {a[2]*b[3]-a[3]*b[2],a[3]*b[1]-a[1]*b[3],a[1]*b[2]-a[2]*b[1]} end
local function unit(a) local n=norm(a);return {a[1]/n,a[2]/n,a[3]/n} end
local function clamp(x,a,b) return math.max(a,math.min(b,x)) end
function windAt(p,t,speed,angle,gust)
    local layer=0.65+0.35*math.exp(-((p[3]-3.6)/2.5)^2)
    local hot=0
    for _,q in ipairs({{-4,-4,3,1.8,1.05},{4,-4,3,1.8,1.05},{0,4,3.3,1.8,1.2}}) do
        hot=hot+q[5]*math.exp(-((p[1]-q[1])/q[3])^2-((p[2]-q[2])/q[4])^2)
    end
    local gain=(0.32+hot)*layer
    if gust then gain=gain*(1+0.15*math.sin(1.3*t+0.21*p[1])+0.08*math.sin(2.7*t+0.3*p[2])) end
    local r=angle*math.pi/180
    return {speed*gain*math.cos(r),speed*gain*math.sin(r),0}
end
function sysCall_init()
    H=sim.getObject('/InspectionDrone')
    ref=sim.getObjectPosition(H);refv={0,0,0};refa={0,0,0};yaw=0;integ={0,0,0};last={}
    trail=sim.addDrawingObject(sim.drawing_linestrip+sim.drawing_painttag,3,0,-1,30000,{0.1,0.85,0.8})
    reftrail=sim.addDrawingObject(sim.drawing_linestrip+sim.drawing_painttag,2,0,-1,30000,{1,0.65,0.16})
    arrows=sim.addDrawingObject(sim.drawing_lines+sim.drawing_painttag,3,0,-1,200,{0.99,0.62,0.22});lastArrow=-10
end
function command(p,v,a,psi,speed,angle)
    ref=p;refv=v;refa=a;yaw=psi;windSpeed=speed;windAngle=angle
    return sample()
end
function sample()
    local p=sim.getObjectPosition(H)
    local v,w=sim.getObjectVelocity(H+sim.handleflag_axis)
    local m=sim.getObjectMatrix(H)
    return {p=p,v=v,omega=w,forward={m[1],m[5],m[9]},up={m[3],m[7],m[11]},
        wind=windAt(p,sim.getSimulationTime(),windSpeed,windAngle,true),
        thrust=last.thrust or 0,torque=last.torque or {0,0,0},force=last.force or {0,0,0},
        time=sim.getSimulationTime()}
end
-- One compact API response for the browser: render and encode inside CoppeliaSim.
-- The '.jpg' destination returns a memory buffer; it does not create a file.
function overview_jpeg()
    local overview=sim.getObject('/OverviewCamera')
    sim.handleVisionSensor(overview)
    local raw,res=sim.getVisionSensorImg(overview)
    return sim.saveImage(raw,res,0,'.jpg',78)
end
function sysCall_actuation()
    local dt=sim.getSimulationTimeStep()
    local p=sim.getObjectPosition(H)
    local v,omega=sim.getObjectVelocity(H+sim.handleflag_axis)
    local m=sim.getObjectMatrix(H)
    local b1={m[1],m[5],m[9]};local b2={m[2],m[6],m[10]};local b3={m[3],m[7],m[11]}
    local acc={}
    for i=1,3 do
        local e=ref[i]-p[i]
        integ[i]=clamp(integ[i]+e*dt,-2.8,2.8)
        local kp=i==3 and 4.5 or 4.5
        local kd=i==3 and 3.8 or 4.0
        local ki=i==3 and 0.65 or 0.85
        acc[i]=clamp(refa[i]+kp*e+kd*(refv[i]-v[i])+ki*integ[i],-4.0,4.0)
    end
    local horizontal=math.sqrt(acc[1]^2+acc[2]^2)
    if horizontal>4 then acc[1]=acc[1]*4/horizontal;acc[2]=acc[2]*4/horizontal end
    local desired={acc[1],acc[2],9.81+acc[3]}
    local d3=unit(desired)
    local d2=unit(cross(d3,{math.cos(yaw),math.sin(yaw),0}))
    local d1=cross(d2,d3)
    local er1=cross(b1,d1);local er2=cross(b2,d2);local er3=cross(b3,d3)
    local alpha={}
    for i=1,3 do alpha[i]=55*0.5*(er1[i]+er2[i]+er3[i])-11*omega[i] end
    local inertia={0.021,0.027,0.041}
    local axes={b1,b2,b3};local torque={0,0,0};local Iomega={0,0,0}
    for j=1,3 do
        local al=dot(alpha,axes[j]);local om=dot(omega,axes[j])
        for i=1,3 do torque[i]=torque[i]+inertia[j]*al*axes[j][i];Iomega[i]=Iomega[i]+inertia[j]*om*axes[j][i] end
    end
    local gyro=cross(omega,Iomega)
    for i=1,3 do torque[i]=clamp(torque[i]+gyro[i],-0.8,0.8) end
    local thrust=clamp(1.5*dot(desired,b3),0,32)
    local wind=windAt(p,sim.getSimulationTime(),windSpeed,windAngle,true)
    local rel={wind[1]-v[1],wind[2]-v[2],wind[3]-v[3]}
    local mag=norm(rel);local drag={};local force={}
    for i=1,3 do drag[i]=0.075*mag*rel[i];force[i]=thrust*b3[i]+drag[i] end
    sim.addForceAndTorque(H,force,torque)
    last={thrust=thrust,torque=torque,force=drag}
end
function sysCall_sensing()
    sim.addDrawingObjectItem(trail,sim.getObjectPosition(H))
    sim.addDrawingObjectItem(reftrail,ref)
    local t=sim.getSimulationTime()
    if t-lastArrow>0.5 then
        sim.addDrawingObjectItem(arrows,nil);lastArrow=t
        for x=-12,12,6 do for y=-7,7,7 do
            local w=windAt({x,y,2},t,windSpeed,windAngle,true)
            local n=math.sqrt(w[1]^2+w[2]^2)
            if n>0.01 then
                local len=math.min(2,0.3*n);local dx=w[1]/n;local dy=w[2]/n
                local ex=x+len*dx;local ey=y+len*dy
                sim.addDrawingObjectItem(arrows,{x,y,1.2,ex,ey,1.2})
                sim.addDrawingObjectItem(arrows,{ex,ey,1.2,ex-0.3*dx+0.15*dy,ey-0.3*dy-0.15*dx,1.2})
                sim.addDrawingObjectItem(arrows,{ex,ey,1.2,ex-0.3*dx-0.15*dy,ey-0.3*dy+0.15*dx,1.2})
            end
        end end
    end
end
