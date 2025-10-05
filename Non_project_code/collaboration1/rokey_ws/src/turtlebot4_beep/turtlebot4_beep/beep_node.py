import subprocess

def main():
    print('Hi from turtlebot4_beep.')
    command = (
        'ros2 topic pub --once /robot2/cmd_audio irobot_create_msgs/msg/AudioNoteVector "{append: false, notes: ['
        '{frequency: 880.0, max_runtime: {sec: 0, nanosec: 300000000}}, '
        '{frequency: 440.0, max_runtime: {sec: 0, nanosec: 300000000}}, '
        '{frequency: 880.0, max_runtime: {sec: 0, nanosec: 300000000}}, '
        '{frequency: 440.0, max_runtime: {sec: 0, nanosec: 300000000}}]}"'
        ''
    )
    try:
        subprocess.run(command, shell=True, check=True)
        print("Audio command published successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")

if __name__ == '__main__':
    main()