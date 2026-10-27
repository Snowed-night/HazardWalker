#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path


if __name__ == '__main__':
    from generate_competition_scene import main as _generate_competition_scene

    raw_args = sys.argv[1:]
    if raw_args and not raw_args[0].startswith("-"):
        output_dir = Path(raw_args[0])
        floor_count = raw_args[1] if len(raw_args) > 1 else "3"
        rooms_per_floor = raw_args[2] if len(raw_args) > 2 else "4"
        converted_args = [
            "--output-dir",
            str(output_dir),
            "--results-dir",
            str(output_dir.parent / "results"),
            "--floor-count",
            str(floor_count),
            "--rooms-per-floor",
            str(rooms_per_floor),
        ]
        raise SystemExit(_generate_competition_scene(converted_args))
    raise SystemExit(_generate_competition_scene(raw_args))

import os
import json

def generate_building(num_floors=3, rooms_per_floor=3):
    floor_height = 1.5
    room_width = 3.0
    room_depth = 3.0
    corridor_width = 2.0
    wall_thickness = 0.1
    stair_width = 1.5
    door_width = 0.8
    
    building_width = rooms_per_floor * room_width + (rooms_per_floor + 1) * wall_thickness
    building_depth = corridor_width + 2 * room_depth + 2 * wall_thickness + stair_width
    
    half_depth = building_depth / 2
    half_width = building_width / 2
    
    sdf = f"""<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='MultiFloorBuilding'>
    <static>true</static>

    <!-- Ground base -->
    <link name='ground_base'>
      <pose>0 0 -0.15 0 0 0</pose>
      <visual name='visual'>
        <geometry><box><size>{building_width+1} {building_depth+1} 0.2</size></box></geometry>
        <material><ambient>0.3 0.4 0.3 1</ambient><diffuse>0.4 0.5 0.4 1</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{building_width+1} {building_depth+1} 0.2</size></box></geometry>
      </collision>
    </link>
"""

    for floor in range(num_floors):
        z = floor * floor_height
        
        # Floor platform
        sdf += f"""
    <!-- Floor {floor} -->
    <link name='floor_{floor}'>
      <pose>0 0 {z + 0.05} 0 0 0</pose>
      <visual name='visual'>
        <geometry><box><size>{building_width} {building_depth} 0.1</size></box></geometry>
        <material><ambient>0.8 0.75 0.7 1</ambient><diffuse>0.85 0.8 0.75 1</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{building_width} {building_depth} 0.1</size></box></geometry>
      </collision>
    </link>

    <!-- Outer walls -->
    <link name='outer_wall_north_{floor}'>
      <pose>0 {half_depth - wall_thickness/2} {z + floor_height/2} 0 0 0</pose>
      <visual name='visual'>
        <geometry><box><size>{building_width} {wall_thickness} {floor_height}</size></box></geometry>
        <material><ambient>0.55 0.5 0.45 1</ambient><diffuse>0.65 0.6 0.55 1</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{building_width} {wall_thickness} {floor_height}</size></box></geometry>
      </collision>
    </link>

    <link name='outer_wall_south_{floor}'>
      <pose>0 {-half_depth + wall_thickness/2} {z + floor_height/2} 0 0 0</pose>
      <visual name='visual'>
        <geometry><box><size>{building_width} {wall_thickness} {floor_height}</size></box></geometry>
        <material><ambient>0.55 0.5 0.45 1</ambient><diffuse>0.65 0.6 0.55 1</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{building_width} {wall_thickness} {floor_height}</size></box></geometry>
      </collision>
    </link>

    <link name='outer_wall_west_{floor}'>
      <pose>{-half_width + wall_thickness/2} 0 {z + floor_height/2} 0 0 0</pose>
      <visual name='visual'>
        <geometry><box><size>{wall_thickness} {building_depth - stair_width - wall_thickness} {floor_height}</size></box></geometry>
        <material><ambient>0.55 0.5 0.45 1</ambient><diffuse>0.65 0.6 0.55 1</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{wall_thickness} {building_depth - stair_width - wall_thickness} {floor_height}</size></box></geometry>
      </collision>
    </link>

    <!-- East outer wall with entrance gap -->
    <link name='outer_wall_east_{floor}'>
      <pose>{half_width - wall_thickness/2} 0 {z + floor_height/2} 0 0 0</pose>
      <visual name='visual'>
        <geometry><box><size>{wall_thickness} {building_depth - 2*door_width} {floor_height}</size></box></geometry>
        <material><ambient>0.55 0.5 0.45 1</ambient><diffuse>0.65 0.6 0.55 1</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{wall_thickness} {building_depth - 2*door_width} {floor_height}</size></box></geometry>
      </collision>
    </link>

    <!-- Entrance frame on east wall -->
    <link name='entrance_top_{floor}'>
      <pose>{half_width - wall_thickness/2} {half_depth - wall_thickness - door_width/2} {z + floor_height/2} 0 0 0</pose>
      <visual name='visual'>
        <geometry><box><size>{wall_thickness} {door_width} {floor_height}</size></box></geometry>
        <material><ambient>0.5 0.4 0.3 1</ambient><diffuse>0.6 0.5 0.4 1</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{wall_thickness} {door_width} {floor_height}</size></box></geometry>
      </collision>
    </link>

    <link name='entrance_bottom_{floor}'>
      <pose>{half_width - wall_thickness/2} {-half_depth + wall_thickness + door_width/2} {z + floor_height/2} 0 0 0</pose>
      <visual name='visual'>
        <geometry><box><size>{wall_thickness} {door_width} {floor_height}</size></box></geometry>
        <material><ambient>0.5 0.4 0.3 1</ambient><diffuse>0.6 0.5 0.4 1</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{wall_thickness} {door_width} {floor_height}</size></box></geometry>
      </collision>
    </link>

    <!-- Room divider walls -->
"""
        for room in range(rooms_per_floor - 1):
            room_div_x = -half_width + wall_thickness + room_width/2 + wall_thickness/2 + room*(room_width + wall_thickness)
            
            sdf += f"""
    <link name='room_divider_{floor}_{room}'>
      <pose>{room_div_x} 0 {z + floor_height/2} 0 0 0</pose>
      <visual name='visual'>
        <geometry><box><size>{wall_thickness} {building_depth - 2*wall_thickness} {floor_height}</size></box></geometry>
        <material><ambient>0.6 0.55 0.5 1</ambient><diffuse>0.7 0.65 0.6 1</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{wall_thickness} {building_depth - 2*wall_thickness} {floor_height}</size></box></geometry>
      </collision>
    </link>
"""

        # North corridor wall with door gaps
        sdf += f"""
    <!-- North corridor wall -->
"""
        for room in range(rooms_per_floor):
            wall_x = -half_width + wall_thickness + room * (room_width + wall_thickness)
            sdf += f"""
    <link name='corridor_wall_north_left_{floor}_{room}'>
      <pose>{wall_x} {room_depth + wall_thickness/2} {z + floor_height/2} 0 0 0</pose>
      <visual name='visual'>
        <geometry><box><size>{(room_width - door_width)/2} {wall_thickness} {floor_height}</size></box></geometry>
        <material><ambient>0.6 0.55 0.5 1</ambient><diffuse>0.7 0.65 0.6 1</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{(room_width - door_width)/2} {wall_thickness} {floor_height}</size></box></geometry>
      </collision>
    </link>

    <link name='corridor_wall_north_right_{floor}_{room}'>
      <pose>{wall_x + door_width + (room_width - door_width)/2} {room_depth + wall_thickness/2} {z + floor_height/2} 0 0 0</pose>
      <visual name='visual'>
        <geometry><box><size>{(room_width - door_width)/2} {wall_thickness} {floor_height}</size></box></geometry>
        <material><ambient>0.6 0.55 0.5 1</ambient><diffuse>0.7 0.65 0.6 1</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{(room_width - door_width)/2} {wall_thickness} {floor_height}</size></box></geometry>
      </collision>
    </link>
"""

        # South corridor wall with door gaps
        sdf += f"""
    <!-- South corridor wall -->
"""
        for room in range(rooms_per_floor):
            wall_x = -half_width + wall_thickness + room * (room_width + wall_thickness)
            sdf += f"""
    <link name='corridor_wall_south_left_{floor}_{room}'>
      <pose>{wall_x} {-room_depth - wall_thickness/2} {z + floor_height/2} 0 0 0</pose>
      <visual name='visual'>
        <geometry><box><size>{(room_width - door_width)/2} {wall_thickness} {floor_height}</size></box></geometry>
        <material><ambient>0.6 0.55 0.5 1</ambient><diffuse>0.7 0.65 0.6 1</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{(room_width - door_width)/2} {wall_thickness} {floor_height}</size></box></geometry>
      </collision>
    </link>

    <link name='corridor_wall_south_right_{floor}_{room}'>
      <pose>{wall_x + door_width + (room_width - door_width)/2} {-room_depth - wall_thickness/2} {z + floor_height/2} 0 0 0</pose>
      <visual name='visual'>
        <geometry><box><size>{(room_width - door_width)/2} {wall_thickness} {floor_height}</size></box></geometry>
        <material><ambient>0.6 0.55 0.5 1</ambient><diffuse>0.7 0.65 0.6 1</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{(room_width - door_width)/2} {wall_thickness} {floor_height}</size></box></geometry>
      </collision>
    </link>
"""

        # Stairs at west end, connecting to corridor
        if floor < num_floors - 1:
            stair_x = -half_width + wall_thickness + stair_width/2
            stair_y = half_depth - wall_thickness - stair_width/2
            num_steps = 10
            step_h = floor_height / num_steps
            step_d = 0.3
            
            for step in range(num_steps):
                step_z = z + 0.1 + step_h/2 + step * step_h
                step_x_local = stair_x + step_d/2 + step * step_d
                
                sdf += f"""
    <link name='stair_{floor}_{step}'>
      <pose>{step_x_local} {stair_y} {step_z} 0 0 0</pose>
      <visual name='visual'>
        <geometry><box><size>{step_d} {stair_width} {step_h}</size></box></geometry>
        <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{step_d} {stair_width} {step_h}</size></box></geometry>
      </collision>
    </link>
"""

    # Roof
    sdf += f"""
    <!-- Roof -->
    <link name='roof'>
      <pose>0 0 {(num_floors-1)*floor_height + 0.15} 0 0 0</pose>
      <visual name='visual'>
        <geometry><box><size>{building_width} {building_depth} 0.1</size></box></geometry>
        <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{building_width} {building_depth} 0.1</size></box></geometry>
      </collision>
    </link>

  </model>
</sdf>
"""
    return sdf, building_width, building_depth

def generate_world():
    return """<?xml version="1.0" ?>
<sdf version="1.5">
    <world name="multi_floor_building">
        <gui fullscreen='false'>
            <camera name='user_camera'>
                <pose>15 -15 12 0 0.4 -1.5</pose>
                <view_controller>orbit</view_controller>
            </camera>
        </gui>
        <physics type="ode">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1</real_time_factor>
            <real_time_update_rate>1000</real_time_update_rate>
            <gravity>0 0 -9.81</gravity>
            <ode>
                <solver><type>quick</type><iters>50</iters><sor>1.3</sor></solver>
                <constraints><cfm>0.0</cfm><erp>0.2</erp><contact_max_correcting_vel>10.0</contact_max_correcting_vel><contact_surface_layer>0.001</contact_surface_layer></constraints>
            </ode>
        </physics>
        <scene>
            <sky><clouds><speed>12</speed></clouds></sky>
        </scene>
        <include><uri>model://sun</uri></include>
        <include><uri>model://ground_plane</uri></include>
        <include><uri>model://multi_floor_building</uri><pose>0 0 0 0 0 0</pose></include>
    </world>
</sdf>
"""

def main():
    import sys
    output_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/building"
    num_floors = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    rooms_per_floor = int(sys.argv[3]) if len(sys.argv) > 3 else 3
    
    os.makedirs(output_dir, exist_ok=True)
    
    building_sdf, width, depth = generate_building(num_floors, rooms_per_floor)
    world_sdf = generate_world()
    
    model_dir = os.path.join(output_dir, "models", "multi_floor_building")
    os.makedirs(model_dir, exist_ok=True)
    
    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(building_sdf)
    
    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write("""<?xml version="1.0"?><model><name>Multi-Floor Building</name><version>1.0</version><sdf version='1.7'>model.sdf</sdf></model>""")
    
    with open(os.path.join(output_dir, "multi_floor_building.world"), "w") as f:
        f.write(world_sdf)
    
    config = {"num_floors": num_floors, "rooms_per_floor": rooms_per_floor, "width": width, "depth": depth}
    with open(os.path.join(output_dir, "building_config.json"), "w") as f:
        json.dump(config, f)
    
    print(f"Building generated: {num_floors} floors, {rooms_per_floor} rooms each")
    print(f"Dimensions: {width:.2f}m x {depth:.2f}m x {num_floors*1.5:.2f}m")

if __name__ == '__main__':
    main()
