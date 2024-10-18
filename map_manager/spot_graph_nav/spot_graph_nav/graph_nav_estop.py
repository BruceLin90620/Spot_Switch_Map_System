import threading
import sys
import queue

import graph_nav_single_map
import estop_gui

if __name__ == '__main__':
    # Create a queue for inter-thread communication
    command_queue = queue.Queue()

    # Create and start the mission_monitor thread
    graph_nav_thread = threading.Thread(target=graph_nav_single_map.main)
    graph_nav_thread.start()

    # Run the estop_gui
    if not estop_gui.main():
        
        # Wait for other threads to finish
        graph_nav_thread.join()
        
        # Exit the program with an error code
        sys.exit(1)

    # Wait for other threads to finish before exiting
    graph_nav_thread.join()

    print("All threads have completed. Exiting main program.")