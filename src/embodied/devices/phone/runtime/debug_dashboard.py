"""Read-only LAN dashboard for live teleoperation diagnostics."""

# The embedded HTML is intentionally kept as one deployable Python module.
# ruff: noqa: E501

from __future__ import annotations

import json
import threading
import time
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from embodied.core.project import PROJECT_ROOT
from typing import Any
from urllib.parse import unquote, urlsplit

from embodied.devices.phone.core.models import (
    Pose,
    RobotCommand,
    RobotState,
    RobotTarget,
    TeleopInput,
)


def _pose(pose: Pose | None) -> dict[str, Any] | None:
    if pose is None:
        return None
    return {
        "position_m": [pose.position.x, pose.position.y, pose.position.z],
        "quaternion_xyzw": [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
    }


class DebugTelemetry:
    """Small lock-protected latest-value store; never blocks the control loop."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._input: TeleopInput | None = None
        self._robot: RobotState | None = None
        self._target: RobotTarget | None = None
        self._command: RobotCommand | None = None
        self._input_at_ns: int | None = None
        self._robot_at_ns: int | None = None
        self._cycle_at_ns: int | None = None
        self._events: list[str] = []
        self._servo_error: str | None = None
        self._receiver_error: str | None = None

    def record_input(self, input_state: TeleopInput) -> None:
        with self._lock:
            self._input = input_state
            self._input_at_ns = time.monotonic_ns()

    def record_cycle(
        self,
        robot_state: RobotState,
        target: RobotTarget,
        command: RobotCommand,
    ) -> None:
        now = time.monotonic_ns()
        with self._lock:
            self._robot = robot_state
            self._target = target
            self._command = command
            self._robot_at_ns = now
            self._cycle_at_ns = now

    def record_event(self, event: str) -> None:
        with self._lock:
            self._events.append(event)
            del self._events[:-24]

    def record_errors(self, *, servo: str | None, receiver: str | None) -> None:
        with self._lock:
            self._servo_error = servo
            self._receiver_error = receiver

    def snapshot(self, *, state: str, metrics: dict[str, Any]) -> dict[str, Any]:
        now = time.monotonic_ns()
        with self._lock:
            input_state = self._input
            robot = self._robot
            target = self._target
            command = self._command
            input_age = (
                None
                if self._input_at_ns is None
                else max(0.0, (now - self._input_at_ns) / 1_000_000.0)
            )
            robot_age = (
                None
                if self._robot_at_ns is None
                else max(0.0, (now - self._robot_at_ns) / 1_000_000.0)
            )
            return {
                "server_time_ns": now,
                "runtime": {
                    "state": state,
                    "input_age_ms": input_age,
                    "robot_age_ms": robot_age,
                    "servo_error": self._servo_error,
                    "receiver_error": self._receiver_error,
                    "events": list(self._events),
                },
                "phone": None
                if input_state is None
                else {
                    "sequence": input_state.sequence,
                    "session_id": input_state.session_id,
                    "timestamp_ns": input_state.timestamp_ns,
                    "pose": _pose(input_state.pose),
                    "linear_velocity": None
                    if input_state.linear_velocity is None
                    else list(input_state.linear_velocity.as_tuple()),
                    "angular_velocity": None
                    if input_state.angular_velocity is None
                    else list(input_state.angular_velocity.as_tuple()),
                    "tracking_quality": input_state.tracking_quality.value,
                    "clutch": input_state.clutch_active,
                    "home": input_state.home_requested,
                    "joint_zero": input_state.joint_zero_requested,
                    "recenter": input_state.recenter_requested,
                    "gripper": input_state.gripper,
                },
                "robot": None
                if robot is None
                else {
                    "timestamp_ns": robot.timestamp_ns,
                    "tcp_pose": _pose(robot.tcp_pose),
                    "joint_position_rad": list(robot.joint_position),
                    "joint_velocity_rad_s": list(robot.joint_velocity),
                    "joint_torque_nm": None
                    if robot.joint_torque is None
                    else list(robot.joint_torque),
                    "gripper": robot.gripper,
                },
                "target": None
                if target is None
                else {
                    "mode": target.mode.value,
                    "pose": _pose(target.target_pose),
                    "gripper": target.gripper,
                    "velocity_scale": target.velocity_scale,
                },
                "command": None
                if command is None
                else {
                    "mode": command.mode.value,
                    "reason": command.reason,
                    "gripper": command.gripper,
                },
                "metrics": metrics,
            }


_HTML = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Teleop / Live Diagnostics</title>
<style>
:root{color-scheme:dark;--bg:#0d1114;--panel:#151b20;--line:#2a343b;--muted:#8b9aa4;--text:#e7edf0;--cyan:#53d6df;--amber:#f4b860;--red:#f27373;--green:#83d39a}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:13px/1.4 ui-monospace,SFMono-Regular,Menlo,monospace;letter-spacing:0}
body:before{content:"";position:fixed;inset:0;pointer-events:none;opacity:.18;background-image:linear-gradient(var(--line) 1px,transparent 1px),linear-gradient(90deg,var(--line) 1px,transparent 1px);background-size:32px 32px}
main{position:relative;max-width:1440px;margin:0 auto;padding:22px 24px 34px}.top{display:flex;justify-content:space-between;align-items:flex-end;border-bottom:1px solid var(--line);padding-bottom:16px;margin-bottom:16px}.eyebrow{color:var(--cyan);font-size:11px;letter-spacing:.12em;text-transform:uppercase}.title{font-size:23px;font-weight:600;margin-top:5px}.stamp{color:var(--muted);font-size:11px;text-align:right}.status{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:16px}.pill{border:1px solid var(--line);padding:7px 10px;background:#11171b}.pill b{color:var(--cyan);font-weight:500}.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}.panel{border:1px solid var(--line);background:rgba(21,27,32,.94);padding:15px;min-width:0}.panel h2{font-size:12px;color:var(--muted);font-weight:500;letter-spacing:.1em;text-transform:uppercase;margin:0 0 13px}.panel h2 span{float:right;color:var(--cyan);letter-spacing:0;text-transform:none}.pose{display:grid;grid-template-columns:1fr 1fr;gap:12px}.readout{border-top:2px solid var(--cyan);padding-top:9px}.readout.robot{border-color:var(--amber)}.label{color:var(--muted);font-size:11px;margin-bottom:7px}.xyz{display:grid;grid-template-columns:repeat(3,1fr);gap:5px}.cell{background:#101519;border:1px solid #202a30;padding:8px 7px}.cell i{display:block;color:var(--muted);font-style:normal;font-size:10px}.cell strong{display:block;color:var(--text);font-size:15px;font-weight:500;margin-top:3px}.cell strong.cyan{color:var(--cyan)}.cell strong.amber{color:var(--amber)}.quat{color:var(--muted);font-size:11px;margin-top:9px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}.mapping{grid-column:1/-1}.axis-table{width:100%;border-collapse:collapse}.axis-table th,.axis-table td{padding:8px;border-bottom:1px solid #243038;text-align:left}.axis-table th{color:var(--muted);font-weight:400;font-size:10px;text-transform:uppercase}.axis-table td:nth-child(2){color:var(--cyan)}.axis-table td:nth-child(3){color:var(--amber)}.joint-table{width:100%;border-collapse:collapse;font-size:12px}.joint-table th,.joint-table td{padding:7px 6px;border-bottom:1px solid #243038;text-align:right}.joint-table th:first-child,.joint-table td:first-child{text-align:left;color:var(--muted)}.joint-table th{color:var(--muted);font-weight:400;font-size:10px}.event{color:var(--muted);font-size:11px;line-height:1.7;max-height:145px;overflow:auto;white-space:pre-wrap}.good{color:var(--green)!important}.warn{color:var(--amber)!important}.bad{color:var(--red)!important}.empty{color:var(--muted);padding:25px 0;text-align:center}.wide{grid-column:1/-1}.footer{margin-top:14px;color:var(--muted);font-size:10px}@media(max-width:850px){main{padding:16px 12px}.grid{grid-template-columns:1fr}.mapping,.wide{grid-column:auto}.pose{grid-template-columns:1fr}.top{align-items:flex-start;gap:10px;flex-direction:column}.stamp{text-align:left}}
.visuals{display:grid;grid-template-columns:1fr 1.35fr;gap:12px;margin-bottom:12px}.visual{border:1px solid var(--line);background:rgba(21,27,32,.94);padding:12px}.visual h2{font-size:12px;color:var(--muted);font-weight:500;letter-spacing:.1em;text-transform:uppercase;margin:0 0 8px}.visual canvas{display:block;width:100%;height:230px;background:radial-gradient(circle at 50% 45%,#1c2a30,#101519 72%);border:1px solid #243038}.visual-note{color:var(--muted);font-size:10px;margin-top:7px}@media(max-width:850px){.visuals{grid-template-columns:1fr}}
html,body{height:100%;overflow:hidden}main{height:100vh;display:flex;flex-direction:column;padding:14px 18px;overflow:hidden}.top{margin-bottom:10px;padding-bottom:10px}.title{font-size:20px}.status{margin-bottom:10px;gap:6px}.pill{padding:5px 8px}.visuals{flex:0 0 314px;min-height:0;gap:8px;margin-bottom:8px}.visual{min-height:0;padding:9px}.visual h2{margin-bottom:6px}.visual canvas{height:266px}.visual-note{margin-top:4px}.grid{flex:1;min-height:0;grid-template-rows:minmax(105px,.85fr) minmax(62px,.45fr) minmax(105px,.8fr) minmax(72px,.55fr);gap:8px}.panel{padding:10px;overflow:hidden}.panel h2{margin-bottom:8px}.cell{padding:5px}.cell strong{font-size:13px}.joint-table th,.joint-table td{padding:4px}.event{max-height:none;overflow:auto}.footer{display:none}
@media(max-width:850px){html,body{height:auto;overflow:auto}main{height:auto;min-height:100vh;overflow:visible;padding:14px 12px}.visuals{flex:0 0 auto}.visual canvas{height:220px}.grid{flex:0 0 auto;grid-template-rows:auto;overflow:visible}.panel{overflow:visible}.footer{display:block}}
.fault-banner{border:1px solid var(--red);background:rgba(108,32,38,.28);color:#ffb5b5;padding:8px 11px;margin:-2px 0 10px;line-height:1.35;display:flex;gap:10px;align-items:flex-start}.fault-banner[hidden]{display:none}.fault-text{flex:1;white-space:pre-wrap;user-select:text;cursor:copy}.fault-actions{display:flex;flex-direction:column;gap:4px;align-items:flex-end;min-width:76px}
</style></head><body><main>
<div class="dashboard-meta"><span id="stamp">connecting...</span></div>
<div class="status"><div class="pill">RUNTIME <b id="runtime">--</b></div><div class="pill">TRACKING <b id="tracking">--</b></div><div class="pill">CLUTCH <b id="clutch">--</b></div><div class="pill">INPUT AGE <b id="inputAge">--</b></div><div class="pill">ROBOT AGE <b id="robotAge">--</b></div></div>
<div id="faultBanner" class="fault-banner" hidden><span id="faultText" class="fault-text" title="Click to copy current error"></span><div class="fault-actions"><button id="copyFault" class="copy-button" type="button" title="Copy current error">COPY ERROR</button><span id="faultCopyStatus" class="copy-status" aria-live="polite"></span></div></div>
<section class="visuals"><article class="visual"><h2>iPhone 16 Pro Max / table frame</h2><canvas id="phoneScene" width="720" height="460"></canvas><div class="visual-note">top = +X forward · right = -Y · screen normal = +Z</div></article><article class="visual"><h2>ARX5 X5 / encoder FK</h2><canvas id="robotScene" width="900" height="460"></canvas><div class="visual-note">encoder joints + calibrated jaw · amber = eef · cyan = live FK</div></article></section>
<section class="grid"><article class="panel"><h2>Phone pose <span id="phoneSeq">seq --</span></h2><div id="phone" class="empty">waiting for phone packets</div></article>
<article class="panel"><h2>Robot TCP pose <span id="robotTs">sdk --</span></h2><div id="robot" class="empty">waiting for robot feedback</div></article>
<article class="panel mapping"><h2>Motion frame / target</h2><table class="axis-table"><thead><tr><th>phone motion</th><th>mapped base/table axis</th><th>current target delta</th></tr></thead><tbody id="axes"></tbody></table></article>
<article class="panel wide"><h2>Joint state</h2><div id="gripperState" class="gripper-state empty">waiting for gripper feedback</div><div id="joints" class="empty">waiting for robot feedback</div></article>
<article class="panel"><h2>Command</h2><div id="command" class="event">--</div></article><article class="panel"><h2>Recent events / faults <button id="copyEvents" class="copy-button" type="button" title="Copy current faults and events" disabled>COPY</button></h2><div id="events" class="event">--</div><div id="copyStatus" class="copy-status" aria-live="polite"></div></article></section>
<div class="footer">read-only dashboard · refresh 10 Hz · values are live from the control process</div></main>
<script type="module">
import * as THREE from '/static/three.module.min.js';
import { OrbitControls } from '/static/OrbitControls.js';
import { RoundedBoxGeometry } from '/static/RoundedBoxGeometry.js';
import { STLLoader } from '/static/STLLoader.js';

const $=id=>document.getElementById(id); const fmt=(v,d=3)=>v==null?'--':Number(v).toFixed(d); const jointFmt=(v,d=3)=>v==null?'--':`${Number(v)<0?'':'+'}${Number(v).toFixed(d)}`; const vec=v=>v?`<div class="xyz">${['X','Y','Z'].map((n,i)=>`<div class="cell"><i>${n}</i><strong>${fmt(v[i])}</strong></div>`).join('')}</div>`:'<div class="empty">--</div>'; let eventText='',faultText='';
let phoneView,robotView,phoneModel,phoneFrame,robotJoints=[];
const Z_UP_TO_Y_UP=new THREE.Matrix4().makeRotationX(-Math.PI/2);
const PHONE_TO_ROBOT=new THREE.Matrix4().set(0,-1,0,0,1,0,0,0,0,0,1,0,0,0,0,1);
function applyMatrix(object,matrix){const scale=object.scale.clone();matrix.decompose(object.position,object.quaternion,object.scale);object.scale.multiply(scale)}
function poseHtml(p,cls){if(!p)return'<div class="empty">--</div>';return`<div class="readout ${cls||''}"><div class="label">POSITION / m</div>${vec(p.position_m)}<div class="label quat-label">QUATERNION / XYZW</div><div class="quat-grid">${p.quaternion_xyzw.map((v,i)=>`<div class="cell"><i>${['X','Y','Z','W'][i]}</i><strong>${fmt(v,5)}</strong></div>`).join('')}</div></div>`}
function makeView(id,cameraPosition,target){const canvas=$(id),scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(35,1,.01,20),renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:false});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.setClearColor(0x101519);camera.position.fromArray(cameraPosition);const controls=new OrbitControls(camera,canvas);controls.enableDamping=true;controls.enablePan=false;controls.minDistance=.18;controls.maxDistance=3;controls.target.fromArray(target);scene.add(new THREE.HemisphereLight(0xb9edf0,0x11171b,2.2));const key=new THREE.DirectionalLight(0xffffff,2.6);key.position.set(1,1.4,1);scene.add(key);const fill=new THREE.PointLight(0x53d6df,10,3);fill.position.set(-.7,.3,.5);scene.add(fill);return{canvas,scene,camera,renderer,controls}}
function resizeView(view){const width=view.canvas.clientWidth,height=view.canvas.clientHeight;if(!width||!height)return;view.camera.aspect=width/height;view.camera.updateProjectionMatrix();view.renderer.setSize(width,height,false)}
function makePhone(){const group=new THREE.Group(),frame=new THREE.Mesh(new RoundedBoxGeometry(.0776,.163,.00825,7,.003),new THREE.MeshStandardMaterial({color:0x6e797d,metalness:.92,roughness:.24}));group.add(frame);const screen=new THREE.Mesh(new RoundedBoxGeometry(.071,.151,.001,6,.002),new THREE.MeshStandardMaterial({color:0x05090b,metalness:.1,roughness:.15}));screen.position.z=.0048;group.add(screen);const island=new THREE.Mesh(new RoundedBoxGeometry(.027,.007,.0015,5,.003),new THREE.MeshStandardMaterial({color:0x000000}));island.position.set(0,.058,.0056);group.add(island);const islandGlow=new THREE.Mesh(new THREE.PlaneGeometry(.024,.003),new THREE.MeshBasicMaterial({color:0x53d6df,transparent:true,opacity:.35}));islandGlow.position.set(0,.058,.0065);group.add(islandGlow);const bump=new THREE.Mesh(new RoundedBoxGeometry(.034,.034,.0025,5,.006),new THREE.MeshStandardMaterial({color:0x363f43,metalness:.9,roughness:.25}));bump.position.set(-.017,.052,-.0053);group.add(bump);for(const [x,y] of [[-.01,.059],[-.024,.047],[-.01,.035]]){const lens=new THREE.Mesh(new THREE.CylinderGeometry(.006,.006,.003,32),new THREE.MeshStandardMaterial({color:0x071316,metalness:.8,roughness:.08}));lens.rotation.x=Math.PI/2;lens.position.set(x,y,-.007);group.add(lens)}return group}
function loadMesh(loader,name,parent,color){loader.load(`/static/models/arx5/${name}.STL`,geometry=>{geometry.computeVertexNormals();const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color,metalness:.72,roughness:.27}));mesh.castShadow=true;mesh.receiveShadow=true;parent.add(mesh)})}
function frameArrow(parent,direction,color,length=.18){const arrow=new THREE.ArrowHelper(new THREE.Vector3(...direction),new THREE.Vector3(0,0,0),length,color,.025,.012);parent.add(arrow)}
let eefFrame,robotFeedbackFrame,gripperFingers=[];
function ensureEefMarker(){if(eefFrame||!robotFeedbackFrame)return;eefFrame=new THREE.Group();robotFeedbackFrame.add(eefFrame);frameArrow(eefFrame,[1,0,0],0xf4b860,.12);frameArrow(eefFrame,[0,0,1],0x83d39a,.10);const jawMaterial=new THREE.MeshStandardMaterial({color:0xe0e7e9,metalness:.72,roughness:.3}),palm=new THREE.Mesh(new THREE.BoxGeometry(.024,.050,.030),jawMaterial),fingerGeometry=new THREE.BoxGeometry(.065,.009,.016);palm.position.set(.013,0,0);eefFrame.add(palm);for(const sign of [1,-1]){const finger=new THREE.Mesh(fingerGeometry,jawMaterial);finger.position.set(.057,sign*.011,0);eefFrame.add(finger);gripperFingers.push({finger,sign})}}
function updateGripper(open){if(!gripperFingers.length)return;const opening=.010+Math.max(0,Math.min(1,open??0))*.016;for(const {finger,sign} of gripperFingers)finger.position.y=sign*opening}
function jointTransform(group,originRpy,axis,angle){group.quaternion.setFromEuler(new THREE.Euler(...originRpy,'XYZ'));group.rotateOnAxis(new THREE.Vector3(...axis),angle||0)}
function makeRobot(){const root=new THREE.Group(),metal=0xb7c1c4,accent=0xe0e7e9,loader=new STLLoader();root.scale.setScalar(1.35);applyMatrix(root,Z_UP_TO_Y_UP);const j1=new THREE.Group();j1.position.set(0,0,.0603);root.add(j1);const j2=new THREE.Group();j2.position.set(.02,0,.0402);j1.add(j2);const j3=new THREE.Group();j3.position.set(-.264,0,0);j2.add(j3);const j4=new THREE.Group();j4.position.set(.245,0,-.056);j3.add(j4);const j5=new THREE.Group();j5.position.set(.06575,-.001,-.0825);j4.add(j5);const j6=new THREE.Group();j6.position.set(.02845,0,.0825);j5.add(j6);loadMesh(loader,'base_link',root,metal);loadMesh(loader,'link1',j1,metal);loadMesh(loader,'link2',j2,metal);loadMesh(loader,'link3',j3,metal);loadMesh(loader,'link4',j4,accent);loadMesh(loader,'link5',j5,metal);loadMesh(loader,'link6',j6,accent);frameArrow(root,[1,0,0],0xf4b860,.16);frameArrow(root,[0,1,0],0x53d6df,.16);frameArrow(root,[0,0,1],0x83d39a,.16);robotFeedbackFrame=new THREE.Group();frameArrow(robotFeedbackFrame,[1,0,0],0x53d6df,.11);frameArrow(robotFeedbackFrame,[0,0,1],0xf27373,.09);root.add(robotFeedbackFrame);robotJoints=[j1,j2,j3,j4,j5,j6];return root}
function updateModels(phone,robot){if(phoneModel&&phone?.pose){phoneModel.position.fromArray(phone.pose.position_m||[0,0,0]);phoneModel.quaternion.fromArray(phone.pose.quaternion_xyzw||[0,0,0,1])}if(robot&&robotJoints.length){const q=robot.joint_position_rad;jointTransform(robotJoints[0],[0,0,0],[0,0,1],q[0]);jointTransform(robotJoints[1],[0,0,0],[0,1,0],q[1]);jointTransform(robotJoints[2],[-Math.PI,0,0],[0,1,0],q[2]);jointTransform(robotJoints[3],[0,0,0],[0,1,0],q[3]);jointTransform(robotJoints[4],[0,0,0],[0,0,1],q[4]);jointTransform(robotJoints[5],[-Math.PI,0,0],[1,0,0],q[5]);if(robotFeedbackFrame&&robot.tcp_pose){robotFeedbackFrame.position.fromArray(robot.tcp_pose.position_m);robotFeedbackFrame.quaternion.fromArray(robot.tcp_pose.quaternion_xyzw)}}}
function animate(){requestAnimationFrame(animate);for(const view of [phoneView,robotView]){if(!view)continue;resizeView(view);view.controls.update();view.renderer.render(view.scene,view.camera)}}
function initModels(){phoneView=makeView('phoneScene',[.20,.16,.24],[0,0,0]);phoneFrame=new THREE.Group();applyMatrix(phoneFrame,new THREE.Matrix4().multiplyMatrices(Z_UP_TO_Y_UP,PHONE_TO_ROBOT));phoneModel=makePhone();phoneFrame.add(phoneModel);phoneView.scene.add(phoneFrame);const phoneFloor=new THREE.GridHelper(.24,8,0x2a343b,0x202b31);phoneFloor.position.y=-.006;phoneView.scene.add(phoneFloor);frameArrow(phoneFrame,[0,1,0],0xf4b860,.10);robotView=makeView('robotScene',[.62,.46,.62],[0,.16,0]);robotView.scene.add(makeRobot());const robotFloor=new THREE.GridHelper(1.0,12,0x2a343b,0x202b31);robotFloor.position.y=0;robotView.scene.add(robotFloor);animate()}
function render(s){const r=s.runtime||{},p=s.phone,rb=s.robot,t=s.target,c=s.command; $('stamp').textContent=`server ${new Date().toLocaleTimeString()} · ${location.host}`;$('runtime').textContent=r.state||'--';$('runtime').className=r.servo_error||r.receiver_error?'bad':'';$('tracking').textContent=p?.tracking_quality||'--';$('tracking').className=p?.tracking_quality==='normal'?'good':p?'warn':'';$('clutch').textContent=p?(p.clutch?'ENGAGED':'released'):'--';$('clutch').className=p?.clutch?'good':'';$('inputAge').textContent=r.input_age_ms==null?'--':fmt(r.input_age_ms,0)+' ms';$('robotAge').textContent=r.robot_age_ms==null?'--':fmt(r.robot_age_ms,0)+' ms';$('phoneSeq').textContent=p?`seq ${p.sequence} / s${p.session_id}`:'seq --';$('robotTs').textContent=rb?`sdk ${fmt(rb.timestamp_ns/1e9,3)} s`:'sdk --';$('phone').innerHTML=poseHtml(p?.pose,'');$('robot').innerHTML=poseHtml(rb?.tcp_pose,'robot');updateModels(p,rb);updateGripper(rb?.gripper);let pos=rb?.tcp_pose?.position_m||[0,0,0], target=t?.pose?.position_m||pos; let delta=target.map((v,i)=>v-pos[i]); $('axes').innerHTML=[['phone right / left','gripper right / left (eef -Y / +Y)',''+fmt(delta[1])+' m'],['phone top forward / back','gripper forward / back (eef +X / -X)',''+fmt(delta[0])+' m'],['screen up / down','gripper up / down (eef +Z / -Z)',''+fmt(delta[2])+' m']].map(a=>`<tr><td>${a[0]}</td><td>${a[1]}</td><td>${a[2]}</td></tr>`).join('');if(rb){$('gripperState').className='gripper-state';$('gripperState').innerHTML=`<div class="gripper-cell"><i>POSITION / normalized</i><strong>${fmt(rb.gripper)}</strong></div><div class="gripper-cell"><i>TARGET / normalized</i><strong>${fmt(t?.gripper ?? c?.gripper)}</strong></div>`;$('joints').innerHTML=`<table class="joint-table"><thead><tr><th>joint</th>${rb.joint_position_rad.map((_,i)=>`<th>J${i+1}</th>`).join('')}</tr></thead><tbody><tr><td>position / rad</td>${rb.joint_position_rad.map(v=>`<td>${jointFmt(v)}</td>`).join('')}</tr><tr><td>velocity / rad/s</td>${rb.joint_velocity_rad_s.map(v=>`<td>${jointFmt(v)}</td>`).join('')}</tr><tr><td>torque / Nm</td>${(rb.joint_torque_nm||[]).map(v=>`<td>${jointFmt(v)}</td>`).join('')}</tr></tbody></table>`}else{$('gripperState').className='gripper-state empty';$('gripperState').textContent='waiting for gripper feedback';$('joints').innerHTML='<div class="empty">waiting for robot feedback</div>'}$('command').innerHTML=c?`mode <span class="${c.mode==='cartesian_position'?'good':''}">${c.mode}</span><br>gripper ${fmt(c.gripper)}<br>${c.reason||'active target'}`:'--';const faultParts=[r.servo_error&&`SERVO ERROR: ${r.servo_error}`,r.receiver_error&&`RECEIVER ERROR: ${r.receiver_error}`].filter(Boolean);faultText=faultParts.join('\n');eventText=[...faultParts,...(r.events||[])].filter(Boolean).join('\n')||'no recent faults';$('events').textContent=eventText;$('copyEvents').disabled=eventText==='no recent faults';const banner=$('faultBanner');banner.hidden=!faultText;$('faultText').textContent=faultText?`CONTROL PAUSED · ${r.state||'--'}\n${faultText}\nrelease clutch, then press Recover in the phone app`:'';}
async function poll(){try{const res=await fetch('/api/state',{cache:'no-store'});if(!res.ok)throw Error(res.status);render(await res.json())}catch(e){$('stamp').textContent='dashboard disconnected';$('runtime').textContent='offline';$('runtime').className='bad'}setTimeout(poll,100)}initModels();poll();
setTimeout(ensureEefMarker,0);
async function copyText(value,status='copyStatus'){if(!value||value==='no recent faults')return;try{await navigator.clipboard.writeText(value);$(status).textContent='copied';}catch(error){const area=document.createElement('textarea');area.value=value;document.body.appendChild(area);area.select();document.execCommand('copy');area.remove();$(status).textContent='copied';}setTimeout(()=>{$(status).textContent=''},1200)}
async function copyCurrentEvents(){await copyText(eventText)}async function copyCurrentFault(){await copyText($('faultText').textContent||faultText,'faultCopyStatus')}$('copyEvents').addEventListener('click',copyCurrentEvents);$('copyFault').addEventListener('click',copyCurrentFault);$('faultText').addEventListener('click',copyCurrentFault);
</script></body></html>"""

# Keep the mapping panel readable when the compact dashboard grid is active.
_HTML = _HTML.replace(
    "</style>",
    """<style>
.mapping { overflow: auto !important; }
.dashboard-meta {
  flex: 0 0 15px;
  color: var(--muted);
  font-size: 10px;
  line-height: 15px;
  text-align: right;
}
.status { margin-bottom: 6px; }
.pill { padding: 4px 7px; }
.visuals { flex-basis: 304px; margin-bottom: 6px; }
.visual canvas { height: 256px; }
.visual-note { font-size: 9px; }
.panel { padding: 8px; }
.panel h2 { margin-bottom: 6px; }
.quat {
  white-space: normal;
  overflow: visible;
  text-overflow: clip;
  overflow-wrap: anywhere;
  line-height: 1.35;
}
.quat-label { margin-top: 9px; margin-bottom: 5px; }
.quat-grid { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 5px; }
.quat-grid .cell { min-width: 0; }
.quat-grid .cell strong { font-size: 12px; }
.gripper-state { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 6px; margin-bottom: 7px; }
.gripper-cell { background: #101519; border: 1px solid #202a30; padding: 5px 7px; min-width: 0; }
.gripper-cell i { display: block; color: var(--muted); font-size: 10px; font-style: normal; }
.gripper-cell strong { display: block; color: var(--amber); font-size: 14px; font-weight: 500; margin-top: 2px; }
.copy-button { float: right; border: 1px solid var(--line); background: #101519; color: var(--cyan); font: inherit; font-size: 10px; padding: 2px 6px; cursor: pointer; }
.copy-button:disabled { color: var(--muted); cursor: not-allowed; opacity: .55; }
.copy-status { color: var(--green); font-size: 10px; min-height: 14px; margin-top: 4px; }
.axis-table { table-layout: fixed; }
.axis-table th, .axis-table td {
  padding: 3px 6px;
  line-height: 1.25;
  white-space: normal;
  overflow-wrap: anywhere;
}
.axis-table th:nth-child(1), .axis-table td:nth-child(1) { width: 30%; }
.axis-table th:nth-child(2), .axis-table td:nth-child(2) { width: 45%; }
.axis-table th:nth-child(3), .axis-table td:nth-child(3) {
  width: 25%;
  white-space: nowrap;
}
.joint-table {
  table-layout: fixed;
  font-variant-numeric: tabular-nums;
  font-feature-settings: "tnum" 1;
}
.joint-table th, .joint-table td {
  width: 12.5%;
  padding: 4px 5px;
  overflow: hidden;
  text-overflow: clip;
  white-space: nowrap;
}
.joint-table th:first-child, .joint-table td:first-child { width: 25%; }
.grid {
  grid-template-rows: minmax(112px, .75fr) minmax(104px, .6fr) minmax(128px, 1.05fr) minmax(58px, .45fr);
}
@media (max-width: 850px) {
  .mapping { overflow: visible !important; }
  .axis-table { table-layout: auto; }
  .axis-table th, .axis-table td { padding: 6px 4px; }
}
</style>""",
    1,
)

_HTML = _HTML.replace(
    "updateModels(p,rb);let pos=",
    "updateModels(p,rb);updateGripper(rb?.gripper);let pos=",
    1,
)
_HTML = _HTML.replace(
    "rb.joint_position_rad.map(v=>`<td>${fmt(v)}</td>`)",
    "rb.joint_position_rad.map(v=>`<td>${jointFmt(v)}</td>`)",
    1,
)
_HTML = _HTML.replace(
    "rb.joint_velocity_rad_s.map(v=>`<td>${fmt(v)}</td>`)",
    "rb.joint_velocity_rad_s.map(v=>`<td>${jointFmt(v)}</td>`)",
    1,
)
_HTML = _HTML.replace(
    "(rb.joint_torque_nm||[]).map(v=>`<td>${fmt(v)}</td>`)",
    "(rb.joint_torque_nm||[]).map(v=>`<td>${jointFmt(v)}</td>`)",
    1,
)


class DebugDashboardServer:
    def __init__(
        self,
        telemetry: DebugTelemetry,
        *,
        state_provider: Callable[[], str],
        metrics_provider: Callable[[], dict[str, Any]],
        host: str = "127.0.0.1",
        port: int = 8080,
    ) -> None:
        self._telemetry = telemetry
        self._state_provider = state_provider
        self._metrics_provider = metrics_provider
        self._server = ThreadingHTTPServer((host, port), self._handler())
        self._thread: threading.Thread | None = None

    @property
    def address(self) -> tuple[str, int]:
        return self._server.server_address[:2]

    def _handler(self):
        telemetry = self._telemetry
        state_provider = self._state_provider
        metrics_provider = self._metrics_provider
        static_root = PROJECT_ROOT / "assets" / "phone" / "dashboard"

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                request_path = urlsplit(self.path).path
                if request_path == "/api/state":
                    payload = telemetry.snapshot(
                        state=state_provider(), metrics=metrics_provider()
                    )
                    body = json.dumps(payload, separators=(",", ":")).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-store")
                elif request_path in ("/", "/index.html"):
                    body = _HTML.encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                elif request_path.startswith("/static/"):
                    relative = unquote(request_path.removeprefix("/static/"))
                    candidate = (static_root / relative).resolve()
                    if static_root not in candidate.parents or not candidate.is_file():
                        self.send_error(404)
                        return
                    body = candidate.read_bytes()
                    content_type = (
                        "application/javascript"
                        if candidate.suffix == ".js"
                        else "application/octet-stream"
                    )
                    self.send_response(200)
                    self.send_header("Content-Type", content_type)
                    self.send_header("Cache-Control", "public, max-age=3600")
                else:
                    self.send_error(404)
                    return
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, format: str, *args: object) -> None:
                del format, args

        return Handler

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="robot-teleop-debug-web",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(2.0)
        self._thread = None
