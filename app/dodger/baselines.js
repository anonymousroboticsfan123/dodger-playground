import { solveBoxConstrainedQP } from '../qp.js?v=dodger-3';
import { DODGER_CONFIG as C, clamp, integrateCommand } from './config.js?v=dodger-3';
import { selectObstacles } from './observations.js?v=dodger-3';
import { evaluateBarrier, dpcbfOverlay } from './safety.js?v=dodger-3';

const dot = (a,b) => a.reduce((sum,v,i)=>sum+v*b[i],0);

export function accelerationColumns(robot) {
    const c=Math.cos(robot.yaw),s=Math.sin(robot.yaw);
    return [[c,s],[-s,c],[-robot.worldVelocity[1],robot.worldVelocity[0]]];
}

function relativeData(robot, obstacle, config) {
    const p=[obstacle.x-robot.position[0],obstacle.y-robot.position[1]];
    const v=[obstacle.vx-robot.worldVelocity[0],obstacle.vy-robot.worldVelocity[1]];
    const radius=(config.robotRadius+obstacle.radius)*config.safetyScale;
    return {p,v,radius,physicalClearance:Math.hypot(...p)-config.robotRadius-obstacle.radius};
}

export function evaluateDistanceBarrier(robot, obstacle, action=[0,0,0], config=C) {
    const {p,v,radius,physicalClearance}=relativeData(robot,obstacle,config);
    const h=dot(p,p)-radius*radius, hDot=2*dot(p,v);
    const lgH=accelerationColumns(robot).map(column=>-2*dot(p,column));
    const lfH=2*dot(v,v);
    const offset=lfH+(config.distanceAlpha1+config.distanceAlpha2)*hDot+config.distanceAlpha1*config.distanceAlpha2*h;
    return {h,hDot,lfH,lgH,offset,condition:offset+dot(lgH,action),relativeVelocity:v,physicalClearance,safetyRadius:radius};
}

export function evaluateCollisionCone(robot, obstacle, action=[0,0,0], config=C) {
    const {p,v,radius,physicalClearance}=relativeData(robot,obstacle,config);
    const speed=Math.hypot(...v), dSquared=dot(p,p)-radius*radius;
    const separation=Math.sqrt(Math.max(dSquared,1e-8));
    const h=dot(p,v)+speed*separation;

    const dVelocity=p.map((value,i)=>value+(speed>1e-8?separation*v[i]/speed:0));
    const dPosition=v.map((value,i)=>value+(dSquared>1e-8?speed*p[i]/separation:0));
    const lgH=accelerationColumns(robot).map(column=>-dot(dVelocity,column));
    const lfH=dot(dPosition,v),offset=lfH+config.alpha*h;
    return {h,lfH,lgH,offset,condition:offset+dot(lgH,action),relativeVelocity:v,physicalClearance,safetyRadius:radius};
}

export function analyticActionBounds(command,dt=C.controlDt,config=C) {
    return {
        min:[Math.max(config.actionMin[0],(config.commandMin[0]-command[0])/dt),Math.max(config.actionMin[1],(config.commandMin[1]-command[1])/dt),config.actionMin[2]],
        max:[Math.min(config.actionMax[0],(config.commandMax[0]-command[0])/dt),Math.min(config.actionMax[1],(config.commandMax[1]-command[1])/dt),config.actionMax[2]]
    };
}

export function solveAnalyticController(method,robot,obstacles,command,nominalAction,config=C) {
    const selected=selectObstacles(robot,obstacles,config);
    const evaluate=method==='distance_cbf'?evaluateDistanceBarrier:method==='c3bf'?evaluateCollisionCone:evaluateBarrier;
    const values=selected.map(obstacle=>evaluate(robot,obstacle,nominalAction,config));
    const bounds=analyticActionBounds(command,config.controlDt,config);
    const constraints=values.map((v,i)=>{
        const offset=v.offset??v.lfH+config.alpha*v.h;
        const scale=Math.max(1,Math.hypot(...v.lgH));
        return {label:String(selected[i].id),A:v.lgH.map(x=>x/scale),b:offset/scale};
    });
    const bounded=nominalAction.map((v,i)=>clamp(v,bounds.min[i],bounds.max[i]));

    const unchanged=bounded.every((v,i)=>v===nominalAction[i])&&constraints.every(row=>dot(row.A,bounded)+row.b>=0);
    const start=performance.now();
    const result=unchanged?{u:bounded,feasible:true,status:'optimal',activeSet:[],minMargin:Math.min(Infinity,...constraints.map(row=>dot(row.A,bounded)+row.b))}
        :solveBoxConstrainedQP(nominalAction,constraints,bounds,{weights:[1,1,1],slack:{enabled:false}});
    return {...result,safeAction:result.u,selected,values,constraints,bounds,solveMs:performance.now()-start,
        failed:!result.feasible,slack:new Array(values.length).fill(0),residual:result.feasible?Math.max(0,-result.minMargin):null};
}

function analyticOverlay(method,robot,obstacle,value) {

    let points=[];
    if(method!=='distance_cbf') {
        const bearing=Math.atan2(obstacle.y-robot.position[1],obstacle.x-robot.position[0]);
        const distance=Math.hypot(obstacle.x-robot.position[0],obstacle.y-robot.position[1]);
        const halfAngle=Math.asin(Math.min(1,value.safetyRadius/Math.max(distance,1e-8)));
        const ray=angle=>[robot.position[0]+1.8*Math.cos(angle),robot.position[1]+1.8*Math.sin(angle)];

        points=[ray(bearing+Math.PI-halfAngle),robot.position.slice(0,2),ray(bearing+Math.PI+halfAngle)];
    }
    return {type:method==='distance_cbf'?'distance':'collision-cone',obstacleId:obstacle.id,points,unsafe:value.h<0,
        unsafeFill:method==='c3bf'?points:null,
        relativeVelocityArrow:{start:robot.position.slice(0,2),end:[robot.position[0]+value.relativeVelocity[0],robot.position[1]+value.relativeVelocity[1]]}};
}

export function analyticDiagnostics(method,robot,command,action,result,{visible=true,referenceOnly=false}={}) {
    const referenceCommand=integrateCommand(command,result.safeAction,C.controlDt);
    const overlays=result.values.map((v,i)=>({...(method==='dpcbf'?dpcbfOverlay(robot,result.selected[i],v):analyticOverlay(method,robot,result.selected[i],v)),colorIndex:i}));
    const intervention=Math.hypot(...action.map((v,i)=>v-result.safeAction[i]));
    const min=values=>values.length?Math.min(...values):null;
    return {enabled:visible,params:C,overlays,referenceCommand,referenceOnly,referenceAvailable:result.feasible,
        clearance:min(result.values.map(v=>v.physicalClearance)),minBarrier:min(result.values.map(v=>v.h)),
        constrainedObstacleIds:result.selected.map(o=>o.id),predictedPath:[],intervention,
        intervening:!referenceOnly&&result.feasible&&intervention>.012,
        safeAction:[...result.safeAction],qpResidual:result.residual,qpFailed:!result.feasible,
        filterApplied:!referenceOnly,status:result.feasible?(referenceOnly?'reference':'optimal'):'infeasible',
        qpStatus:result.status,solveMs:result.solveMs,
        visualLabel:method==='dpcbf'?'DPCBF':method==='c3bf'?'C3BF':'DIST CBF',
        parameterLabel:method==='distance_cbf'?`α₁ ${C.distanceAlpha1.toFixed(2)} · α₂ ${C.distanceAlpha2.toFixed(2)} · scale ${C.safetyScale.toFixed(2)}`
            :method==='c3bf'?`α ${C.alpha.toFixed(2)} · scale ${C.safetyScale.toFixed(2)}`:null,
        constraints:result.values.map((v,i)=>({...v,obstacleId:result.selected[i].id,nominalMargin:v.condition,
            filteredMargin:(v.offset??v.lfH+C.alpha*v.h)+dot(v.lgH,result.safeAction),slack:0}))
    };
}

export function learnedDiagnostics(method,robot,obstacles,command,action,{visible=true}={}) {
    const selected=selectObstacles(robot,obstacles,C);
    const evaluate=method==='distance_cbf'?evaluateDistanceBarrier:method==='c3bf'?evaluateCollisionCone:evaluateBarrier;
    const values=selected.map(obstacle=>evaluate(robot,obstacle,action,C));
    const diagnostics=analyticDiagnostics(method,robot,command,action,{
        selected,values,safeAction:action,feasible:true,residual:null,status:null,solveMs:0
    },{visible});
    return {...diagnostics,referenceCommand:[...command],referenceAvailable:false,showFilteredCommand:false,
        filterApplied:false,intervening:false,intervention:0,qpFailed:false,qpStatus:null,status:'policy',
        constraints:diagnostics.constraints.map(value=>({...value,filteredMargin:null}))};
}
