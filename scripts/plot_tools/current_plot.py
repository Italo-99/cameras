#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import signal
import sys
import csv
import os
import roslib.packages


class CurrentSensorsMonitor:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('current_sensors_monitor_node', anonymous=True)

        # Get the path to your ROS package
        package_path = roslib.packages.get_pkg_dir('sirio_utilities')

        # Directory to store images and counters
        self.save_directory = os.path.join(package_path, 'current_plots')

        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)
        
        # Create a subscriber to the current sensors topic
        self.subscriber = rospy.Subscriber('/sirio/energy/current_sensors', Float32MultiArray, self.current_sensors_callback)

        # Data lists for plotting
        self.time_data = []
        self.converter_data = []
        self.spare_data = []
        self.traction_data = []
        self.steer_data = []

        # Initialize time
        self.start_time = rospy.get_time()

        # Create a single figure with subplots in a grid pattern (2x2 grid)
        self.fig, self.axs = plt.subplots(2, 2, figsize=(10, 8), sharex=True)

        # Titles and labels for each subplot
        self.axs[0, 0].set_title('Converter Sensor [A]')
        self.axs[0, 1].set_title('Spare Sensor [A]')
        self.axs[1, 0].set_title('Traction Sensor [A]')
        self.axs[1, 1].set_title('Steer Sensor [A]')

        for ax in self.axs.flat:
            ax.set_xlabel('Time [s]')
            ax.grid(True)

        self.axs[0, 0].set_ylabel('Current [A]')
        self.axs[0, 1].set_ylabel('Current [A]')
        self.axs[1, 0].set_ylabel('Current [A]')
        self.axs[1, 1].set_ylabel('Current [A]')

        # Animation functions to update each plot
        self.ani_converter = FuncAnimation(self.fig, self.update_converter_plot, interval=1000)
        self.ani_spare = FuncAnimation(self.fig, self.update_spare_plot, interval=1000)
        self.ani_traction = FuncAnimation(self.fig, self.update_traction_plot, interval=1000)
        self.ani_steer = FuncAnimation(self.fig, self.update_steer_plot, interval=1000)

    def current_sensors_callback(self, data):
        # Calculate the elapsed time
        elapsed_time = rospy.get_time() - self.start_time

        # Append the elapsed time and each parameter to their respective lists
        self.time_data.append(elapsed_time)
        self.converter_data.append(data.data[0])
        self.spare_data.append(data.data[1])
        self.traction_data.append(data.data[2])
        self.steer_data.append(data.data[3])

    def update_converter_plot(self, frame):
        self.axs[0, 0].clear()
        self.axs[0, 0].plot(self.time_data, self.converter_data, label="Converter Sensor")
        self.axs[0, 0].set_title('Converter Sensor [A]')
        self.axs[0, 0].set_ylabel('Current [A]')
        self.axs[0, 0].legend()
        self.axs[0, 0].grid(True)

    def update_spare_plot(self, frame):
        self.axs[0, 1].clear()
        self.axs[0, 1].plot(self.time_data, self.spare_data, label="Spare Sensor")
        self.axs[0, 1].set_title('Spare Sensor [A]')
        self.axs[0, 1].set_ylabel('Current [A]')
        self.axs[0, 1].legend()
        self.axs[0, 1].grid(True)

    def update_traction_plot(self, frame):
        self.axs[1, 0].clear()
        self.axs[1, 0].plot(self.time_data, self.traction_data, label="Traction Sensor")
        self.axs[1, 0].set_title('Traction Sensor [A]')
        self.axs[1, 0].set_ylabel('Current [A]')
        self.axs[1, 0].legend()
        self.axs[1, 0].grid(True)

    def update_steer_plot(self, frame):
        self.axs[1, 1].clear()
        self.axs[1, 1].plot(self.time_data, self.steer_data, label="Steer Sensor")
        self.axs[1, 1].set_title('Steer Sensor [A]')
        self.axs[1, 1].set_ylabel('Current [A]')
        self.axs[1, 1].legend()
        self.axs[1, 1].grid(True)
    def save_data(self):
        # Save the data to a CSV file
        with open(self.save_directory + '/current_data.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time [s]', 'Converter Sensor [A]', 'Spare Sensor [A]', 'Traction Sensor [A]', 'Steer Sensor [A]'])
            for i in range(len(self.time_data)):
                writer.writerow([self.time_data[i], self.converter_data[i], self.spare_data[i], self.traction_data[i], self.steer_data[i]])
        rospy.loginfo("Data saved to current_data.csv.")
        
    def save_plot(self):
        # Save the plot to a PNG file
        self.fig.savefig(self.save_directory + '/current_plot.png')
        rospy.loginfo("Plot saved to current_plot.png.")

        # Save each subplot separately
        plot_titles = ['Converter Sensor', 'Spare Sensor', 'Traction Sensor', 'Steer Sensor']
        self.time_data.pop(len(self.time_data) - 1)
        for i, ax in enumerate(self.axs.flat):
            if ax.get_visible():  # Only save if the subplot is visible
                title = plot_titles[i]
                fig_single, ax_single = plt.subplots(figsize=(6, 4))
                for line in ax.get_lines():
                    ax_single.plot(self.time_data, line.get_ydata(), label=line.get_label())
                ax_single.set_title(ax.get_title())
                ax_single.set_xlabel('Time [s]')
                ax_single.set_ylabel(ax.get_ylabel())
                ax_single.legend()
                ax_single.grid(True)
                fig_single.savefig(f"{self.save_directory}/{title}.png")
                plt.close(fig_single)
                rospy.loginfo(f"{title} plot saved to {title}.png")
    def start(self):
        try:
            # Start the matplotlib event loop
            plt.show()
        except KeyboardInterrupt:
            # Handle Ctrl+C to close the plot window
            rospy.loginfo("Shutting down Current Sensors Monitor.")
            plt.close(self.fig)
            sys.exit(0)
    

    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('current_sensors_monitor_node', anonymous=True)
    
    # Create an instance of the CurrentSensorsMonitor class
    monitor = CurrentSensorsMonitor()

    # Set up signal handler for clean shutdown
    def signal_handler(sig, frame):
        rospy.loginfo("Shutting down Current Sensors Monitor due to signal.")
        monitor.save_data()  # Save data before shutting down
        monitor.save_plot()  # Save plot before shutting down
        plt.close(monitor.fig)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        monitor.start()
        # Start the matplotlib event loop and ROS node
        monitor.run()
    except rospy.ROSInterruptException:
        pass
