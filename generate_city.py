#!/usr/bin/env python3
import random
import argparse
from pathlib import Path

parser = argparse.ArgumentParser(description="Generate the seeded PALS urban Gazebo world")
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "px4/Tools/simulation/gz/worlds/urban.sdf")
args = parser.parse_args()
sdf = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="urban">
    <physics type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor><real_time_update_rate>250</real_time_update_rate></physics>
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="gz::sim::systems::Contact" filename="gz-sim-contact-system"/>
    <plugin name="gz::sim::systems::Imu" filename="gz-sim-imu-system"/>
    <plugin name="gz::sim::systems::AirPressure" filename="gz-sim-air-pressure-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system"><render_engine>ogre2</render_engine></plugin>

    <gui fullscreen="false">
      <plugin name="3D View" filename="GzScene3D">
        <gz-gui><title>3D View</title><property type="bool" key="showTitleBar">0</property><property type="string" key="state">docked</property></gz-gui>
        <engine>ogre2</engine><scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light><background_color>0.53 0.81 0.98</background_color>
        <camera_pose>-10 0 5 0 0.3 0</camera_pose>
      </plugin>
      
      <!-- Follow the x500 drone automatically on load -->
      <plugin filename="CameraTracking" name="Camera Tracking">
        <gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="state" type="string">floating</property><property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>

      <plugin name="World control" filename="WorldControl">
        <gz-gui><title>World control</title><property type="bool" key="showTitleBar">0</property><property type="bool" key="resizable">0</property><property type="double" key="height">72</property><property type="double" key="width">121</property><property type="double" key="z">1</property><property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors>
        </gz-gui>
        <play_pause>1</play_pause><step>1</step><start_paused>1</start_paused>
      </plugin>
      <plugin name="World stats" filename="WorldStats">
        <gz-gui><title>World stats</title><property type="bool" key="showTitleBar">0</property><property type="bool" key="resizable">0</property><property type="double" key="height">110</property><property type="double" key="width">290</property><property type="double" key="z">1</property><property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="right" target="right"/><line own="bottom" target="bottom"/></anchors>
        </gz-gui>
        <sim_time>1</sim_time><real_time>1</real_time><real_time_factor>1</real_time_factor><iterations>1</iterations>
      </plugin>
      <plugin name="Entity tree" filename="EntityTree"/>
    </gui>

    <gravity>0 0 -9.8</gravity><magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field><atmosphere type="adiabatic"/>
    <scene><sky><clouds><speed>12</speed></clouds></sky><grid>false</grid><ambient>0.4 0.4 0.4 1</ambient><background>0.53 0.81 0.98 1</background><shadows>true</shadows></scene>

    <light name="sun" type="directional"><pose>0 0 500 0 -0 0</pose><cast_shadows>true</cast_shadows><intensity>1</intensity><direction>0.001 0.625 -0.78</direction><diffuse>1.0 0.98 0.9 1</diffuse><specular>0.4 0.4 0.4 1</specular><attenuation><range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation></light>

    <model name="ground_plane"><static>true</static><link name="link"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry></collision><visual name="visual"><geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry><material><ambient>0.25 0.5 0.15 1</ambient><diffuse>0.3 0.6 0.2 1</diffuse><specular>0.05 0.05 0.05 1</specular></material></visual></link></model>
"""

grid_size = 400
block_size = 60
road_width = 10
road_count = 0

for x in range(-grid_size, grid_size + 1, block_size):
    sdf += f'<model name="road_ns_{road_count}"><static>true</static><pose>{x} 0 0.01 0 0 0</pose><link name="link"><visual name="visual"><geometry><box><size>{road_width} {grid_size*2} 0.01</size></box></geometry><material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material></visual></link></model>\n'
    road_count+=1
for y in range(-grid_size, grid_size + 1, block_size):
    sdf += f'<model name="road_ew_{road_count}"><static>true</static><pose>0 {y} 0.01 0 0 0</pose><link name="link"><visual name="visual"><geometry><box><size>{grid_size*2} {road_width} 0.01</size></box></geometry><material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material></visual></link></model>\n'
    road_count+=1

colors = [(0.5,0.6,0.7), (0.75,0.6,0.45), (0.85,0.75,0.5), (0.6,0.65,0.6), (0.8,0.7,0.55), (0.45,0.55,0.7), (0.65,0.55,0.5), (0.55,0.5,0.45), (0.9,0.8,0.6)]
b_count = 0
t_count = 0
random.seed(args.seed)

for ix in range(-grid_size + block_size//2, grid_size, block_size):
    for iy in range(-grid_size + block_size//2, grid_size, block_size):
        if abs(ix) < block_size and abs(iy) < block_size: continue
        if random.random() < 0.8:
            bw, bl, bh = random.uniform(15,30), random.uniform(15,30), random.uniform(8,80)
            cx, cy, cz = random.choice(colors)
            sdf += f'<model name="b_{b_count}"><static>true</static><pose>{ix} {iy} {bh/2.0} 0 0 {random.uniform(-0.1,0.1)}</pose><link name="link"><visual name="visual"><geometry><box><size>{bw} {bl} {bh}</size></box></geometry><material><ambient>{cx} {cy} {cz} 1</ambient><diffuse>{cx+0.05} {cy+0.05} {cz+0.05} 1</diffuse><specular>0.2 0.2 0.2 1</specular></material></visual><collision name="collision"><geometry><box><size>{bw} {bl} {bh}</size></box></geometry></collision></link></model>\n'
            b_count+=1
        if random.random() < 0.6:
            tx, ty = ix + random.uniform(-20,20), iy + random.uniform(-20,20)
            th, tr, cr, cg = random.uniform(3,6), random.uniform(0.2,0.4), random.uniform(2.5,4.5), random.uniform(0.4,0.6)
            sdf += f'<model name="t_{t_count}"><static>true</static><pose>{tx} {ty} 0 0 0 0</pose><link name="trunk"><visual name="v"><pose>0 0 {th/2} 0 0 0</pose><geometry><cylinder><radius>{tr}</radius><length>{th}</length></cylinder></geometry><material><ambient>0.4 0.25 0.1 1</ambient><diffuse>0.45 0.3 0.15 1</diffuse></material></visual><collision name="c"><pose>0 0 {th/2} 0 0 0</pose><geometry><cylinder><radius>{tr}</radius><length>{th}</length></cylinder></geometry></collision></link><link name="canopy"><pose>0 0 {th+cr*0.6} 0 0 0</pose><visual name="v"><geometry><sphere><radius>{cr}</radius></sphere></geometry><material><ambient>0.1 {cg} 0.1 1</ambient><diffuse>0.15 {cg+0.05} 0.15 1</diffuse></material></visual><collision name="c"><geometry><sphere><radius>{cr}</radius></sphere></geometry></collision></link></model>\n'
            t_count+=1

sdf += '  </world>\n</sdf>'
args.output.parent.mkdir(parents=True, exist_ok=True)
with args.output.open('w') as f:
    f.write(sdf)
print(f'Done! Generated {b_count} buildings and {t_count} trees.')
