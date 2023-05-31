# Multiarm support

We support creating and handling multiple robot arms with this package. Adapting your `xacro` definition is fairly straight forward. Just create a second ur instance and name it differently and set a different `tf_prefix`.


## Example

Lets assume you want to control two UR16e which are left - `ur_left` and right - `ur_right`. Both arms are connected to the same network as your PC. 

A possible example config which lists all the parameters that need to be different for for both arms could look like this. 

| Parameter | left | right | Explanation | 
| --- | --- | --- |  ----- |
| robot_ip | 192.168.1.10 | 192.168.1.11 | IP address of the robot arm|
| tf_prefix | ur_left | ur_right | Prefix which is added to each joint etc. of the arm| 
| reverse_port | 50001 | 50006 | The port on your local PC which the `ur_cap` on the robot will connect to|
| script_sender_port | 50002 | 50007 | See [ur_driver.h](https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/include/ur_client_library/ur/ur_driver.h)  | 
| trajectory_port | 50004 | 50009 | See [ur_driver.h](https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/include/ur_client_library/ur/ur_driver.h)  |
| script_command_port | 50005 | 50010 | See [ur_driver.h](https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/include/ur_client_library/ur/ur_driver.h) |
| kinematics_param_file | kinematics_params_left.yaml | kinematics_params_right.yaml | Kinematics calibration data. See [calibration manual](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/main/ur_calibration) |
| non_blocking_read | true | true | Has to be `true` otherwise the readout will hang and the connection of the other arm will drop |

__The major__ differences to a single robot config are that you need to specify different ports for the reverse connections, you __need__ to set `non_blocking_read = true` and you __need__ to set a different `tf_prefix`.
You might want to increase the `keep_alive_count` as with more robot arms your PC will have more troubles to keep up with the exact 500Hz update rate required by the `ur_cap` running on the robot arm.
Ofcourse you can change all other parameters seperately for both arms if necessary. 

For environments using the ros2control mock interface setting only the `tf_prefix` is enough.

__Hint__: Choose your `tf_prefix` wisely at the beginning. It will be a lot of work changing it afterwards. 



### Xacro example file

An example xacro file could then look like this:

Please note that we explicitly leave out the arguments part from the `ur.urdf.xacro` example file as this does not contribute to the example. Only the common parameters are declared as properties. From my experience the most easy to use approach is to write a yaml file with the parameters for both arms and load these parameters when initiating the `xacro:ur_robot` macro. 


```
    <xacro:property name="ur_type" value="ur16e"/>
    <xacro:property name="joint_limit_params" value="$(find ur_description)/config/${ur_type}/joint_limits.yaml"/>
    <xacro:property name="physical_params" value="$(find ur_description)/config/${ur_type}/physical_parameters.yaml"/>
    <xacro:property name="visual_params" value="$(find ur_description)/config/${ur_type}/visual_parameters.yaml"/>
    <xacro:property name="transmission_hw_interface" value=""/>
    <xacro:property name="safety_limits" value="false"/>
    <xacro:property name="safety_pos_margin" value="0.15"/>
    <xacro:property name="safety_k_position" value="20"/>

    <xacro:property name="use_tool_communication" value="false" />
    <xacro:property name="tool_voltage" value="0" />
    <xacro:property name="tool_parity" value="0" />
    <xacro:property name="tool_baud_rate" value="115200" />
    <xacro:property name="tool_stop_bits" value="1" />
    <xacro:property name="tool_rx_idle_chars" value="1.5" />
    <xacro:property name="tool_tx_idle_chars" value="3.5" />
    <xacro:property name="tool_device_name" value="/tmp/ttyUR" />
    <xacro:property name="tool_tcp_port" value="54321" />


    <xacro:property name="script_filename" value=""/>
    <xacro:property name="output_recipe_filename" value=""/>
    <xacro:property name="input_recipe_filename" value=""/>
    <xacro:property name="reverse_ip" value="0.0.0.0"/>

    <xacro:property name="use_fake_hardware" value="false" />
    <xacro:property name="fake_sensor_commands" value="false" />
    <xacro:property name="sim_gazebo" value="false" />
    <xacro:property name="sim_ignition" value="false" />
    <xacro:property name="simulation_controllers" value="" />
    <xacro:property name="initial_positions_file" value="$(find winder_description)/urdf/initial_positions.yaml"/>



    <xacro:ur_robot name="ur_left" 
        tf_prefix="ur_left/" 
        parent="winder_base_link" 
        joint_limits_parameters_file="${joint_limit_params}" 
        kinematics_parameters_file="${xacro.load_yaml(kinematics_params_left.yaml)}" 
        physical_parameters_file="${physical_params}" 
        visual_parameters_file="${visual_params}" 
        transmission_hw_interface="${transmission_hw_interface}" 
        safety_limits="${safety_limits}" 
        safety_pos_margin="${safety_pos_margin}" 
        safety_k_position="${safety_k_position}" 
        use_fake_hardware="${use_fake_hardware}"
        fake_sensor_commands="${fake_sensor_commands}" 
        sim_gazebo="${sim_gazebo}" 
        sim_ignition="${ sim_ignition}" 
        headless_mode="${ headless_mode}" 
        initial_positions="${xacro.load_yaml(initial_positions_file)}" 
        use_tool_communication="${ use_tool_communication}" 
        tool_voltage="${ tool_voltage}" 
        tool_parity="${ tool_parity}" 
        tool_baud_rate="${ tool_baud_rate}" 
        tool_stop_bits="${ tool_stop_bits}" 
        tool_rx_idle_chars="${ tool_rx_idle_chars}" 
        tool_tx_idle_chars="${ tool_tx_idle_chars}" 
        tool_device_name="${ tool_device_name}" 
        tool_tcp_port="${ tool_tcp_port}" 
        robot_ip="192.168.1.10" 
        script_filename="${ script_filename}"
        output_recipe_filename="${ output_recipe_filename}"
        input_recipe_filename="${ input_recipe_filename}"
        reverse_ip="${ reverse_ip}"
        script_command_port="50005"
        trajectory_port="50004"
        reverse_port="$50001"
        script_sender_port="50002"
        non_blocking_read="true"
        keep_alive_count="10"
        >
      <xacro:insert_block name="origin" />
    </xacro:ur_robot>




    <xacro:ur_robot name="ur_right" 
        tf_prefix="ur_right/" 
        parent="winder_base_link" 
        joint_limits_parameters_file="${joint_limit_params}" 
        kinematics_parameters_file="${kinematics_params_filename}" 
        physical_parameters_file="${physical_params}" 
        visual_parameters_file="${visual_params}" 
        transmission_hw_interface="${transmission_hw_interface}" 
        safety_limits="${safety_limits}" 
        safety_pos_margin="${safety_pos_margin}" 
        safety_k_position="${safety_k_position}" 
        use_fake_hardware="${use_fake_hardware}"
        fake_sensor_commands="${fake_sensor_commands}" 
        sim_gazebo="${sim_gazebo}" 
        sim_ignition="${ sim_ignition}" 
        headless_mode="${ headless_mode}" 
        initial_positions="${xacro.load_yaml(initial_positions_file)}" 
        use_tool_communication="${ use_tool_communication}" 
        tool_voltage="${ tool_voltage}" 
        tool_parity="${ tool_parity}" 
        tool_baud_rate="${ tool_baud_rate}" 
        tool_stop_bits="${ tool_stop_bits}" 
        tool_rx_idle_chars="${ tool_rx_idle_chars}" 
        tool_tx_idle_chars="${ tool_tx_idle_chars}" 
        tool_device_name="${ tool_device_name}" 
        tool_tcp_port="${ tool_tcp_port}" 
        robot_ip="192.168.1.11" 
        script_filename="${ script_filename}"
        output_recipe_filename="${ output_recipe_filename}"
        input_recipe_filename="${ input_recipe_filename}"
        reverse_ip="${ reverse_ip}"
        script_command_port="50010"
        trajectory_port="50009"
        reverse_port="$50006"
        script_sender_port="50007"
        non_blocking_read="true"
        keep_alive_count="10"
        >
      <xacro:insert_block name="origin" />
    </xacro:ur_robot>
```

# Some tips and tricks

From my experience you often want to have different end effectors and tools for both (or even more arms). Therefore it makes sense to put each robot arm (+ everything that is connected to it) into its own `xacro macro` and initiate that macro in your main file. For configurating the arm (something you will play around a lot at the beginning) I decided to create a yaml file which contains the configuration for both arms, load that configuration in the main xacro file and pass the subsection of each arm to the corresponding xacro file. 

The yaml file looks in my case like this:

Note: I do not use the tool communication. Remember to select different device names if you want to use it!

robot_arms.yaml
```
ur_arms:
  ur_left:
    use_fake_hardware: "true"
    prefix: "ur_left/"
    robot_ip: "192.168.1.102"
    script_filename: "resources/ros_control.urscript"
    output_recipe_filename: "resources/rtde_output_recipe.txt"
    input_recipe_filename: "resources/rtde_input_recipe.txt"
    headless_mode : true
    reverse_port: 50001
    script_sender_port: 50002
    servoj_gain: 2000
    servoj_lookahead_time: 0.03
    non_blocking_read: true
    use_tool_communication: false
    trajectory_port: 50004
    script_command_port: 50005
    reverse_ip: "192.168.1.1"
    kinematics_file: "config/ur_16e_left_kinematics.yaml"
    tool:
      tool_tcp_port: 54321
      tool_device_name: "/tmp/ttyUR"
      tool_parity: 0
      tool_baud_rate: 115200
      tool_stop_bits: 1
      tool_rx_idle_chars: 1.5
      tool_tx_idle_chars: 3.5 
      tool_voltage: 24.0
    


  ur_right:
    use_fake_hardware: "true"
    prefix: "ur_right/"
    robot_ip: "192.168.1.101"
    script_filename: "resources/ros_control.urscript"
    output_recipe_filename: "resources/rtde_output_recipe.txt"
    input_recipe_filename: "resources/rtde_input_recipe.txt"
    headless_mode : true
    reverse_port: 50006
    script_sender_port: 50007
    servoj_gain: 2000
    servoj_lookahead_time: 0.03
    non_blocking_read: true
    use_tool_communication: false
    trajectory_port: 50009
    script_command_port: 50010
    reverse_ip: "192.168.1.1"
    kinematics_file: "config/ur_16e_right_kinematics.yaml"
    tool:
      tool_tcp_port: 54321
      tool_device_name: "/tmp/ttyUR"
      tool_parity: 0
      tool_baud_rate: 115200
      tool_stop_bits: 1
      tool_rx_idle_chars: 1.5
      tool_tx_idle_chars: 3.5 
      tool_voltage: 24.0
    


```


In the main xacro I then do:


```
  <xacro:property name="ur_arm_yaml_path" value="$(find my_config)/config/arm_config.yaml"/>
  <xacro:property name="ur_arm_config" value="${xacro.load_yaml(ur_arm_yaml_path)}"/>



  <xacro:ur_bottom prefix="ur_left/" parent="my_parent_link"  config="${ur_arm_config.ur_arms.ur_left}">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:ur_bottom>

  <xacro:ur_top prefix="ur_right/" parent="my_parent_link"  config="${ur_arm_config.ur_arms.ur_right}">
     <origin xyz="1.15 0 0.0" rpy="0 0 0"/>
  </xacro:ur_top>

```