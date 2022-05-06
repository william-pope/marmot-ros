#!/usr/bin/env julia

using RobotOS

@rosimport ackermann_msgs.msg: AckermannDriveStamped
rostypegen()
using .ackermann_msgs.msg

plan_msg = """
using julia
please enter a number to select a driving plan:
1 - drag
2 - shuttle
3 - circle
4 - serpent
5 - steering_test

"""

# is there a class/"self" equivalent for Julia?

function main()
    init_node("preplan_julia")
    ctrl_pub = Publisher{AckermannDriveStamped}("/car/mux/ackermann_cmd_mux/input/manual", queue_size=2)

    while(!is_shutdown())
        op_mode= get_param("/car/mode_select/op_mode")

        if op_mode == "preplan_julia"
            print(plan_msg)
            key_input = readline()

            if key_input == "1"
                plan_name = "drag"

            elseif key_input == "2"
                plan_name = "shuttle"

            elseif key_input == "3"
                plan_name = "circle"

            elseif key_input == "4"
                plan_name = "serpent"

            elseif key_input == "5"
                plan_name = "steering_test"

            else
                print("invalid input")
                continue
            end

            println(plan_name)
            run_plan(ctrl_pub, plan_name)

            break
        else
            continue
        end
    end
end

function run_plan(ctrl_pub, plan_name)
    plan_file = "/home/adcl/catkin_ws/src/manual/src/plans/" * plan_name * ".txt"
    
    plan = []
    open(plan_file, "r") do file
        plan = readlines(file)
    end

    sleep(1.0)
    for c in plan
        ctrl = split(c, ",")
        println(c)
        publish_ctrl(ctrl_pub, ctrl)
    end
end

function publish_ctrl(ctrl_pub, ctrl)
    dur = Duration(1.0)
    rate = Rate(10)
    start = get_rostime()

    ctrl_msg = AckermannDriveStamped()
    ctrl_msg.header.stamp = get_rostime()

    ctrl_msg.drive.speed = parse(Float64, ctrl[1])
    ctrl_msg.drive.steering_angle = parse(Float64, ctrl[2])

    while get_rostime()-start < dur
        publish(ctrl_pub, ctrl_msg)
        sleep(rate)
    end
end

main()