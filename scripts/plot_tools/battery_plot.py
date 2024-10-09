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

class BatteryMonitor:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('battery_monitor_node', anonymous=True)

        # Get the path to your ROS package
        package_path = roslib.packages.get_pkg_dir('sirio_utilities')

        # Directory to store images and counters
        self.save_directory = os.path.join(package_path, 'battery_plots')

        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)
        
        # Create a subscriber to the battery topic
        self.subscriber = rospy.Subscriber('/sirio/energy/battery', Float32MultiArray, self.battery_callback)

        # Data lists for plotting
        self.time_data = []
        self.soc_data = []
        self.temp_data = []
        self.voltage_data = []
        self.current_data = []
        self.capacity_data = []

        # Initialize time
        self.start_time = rospy.get_time()

        # Create a single figure with subplots in a grid pattern (2x3 grid)
        self.fig, self.axs = plt.subplots(3, 2, figsize=(12, 10), sharex=True)

        # Titles and labels for each subplot
        self.axs[0, 0].set_title('State of Charge (SOC) [%]')
        self.axs[0, 1].set_title('Temperature [°C]')
        self.axs[1, 0].set_title('Voltage [V]')
        self.axs[1, 1].set_title('Current [A]')
        self.axs[2, 0].set_title('Residual Capacity [mAh]')
        self.axs[2, 1].set_visible(False)  # Empty subplot

        for ax in self.axs.flat:
            ax.set_xlabel('Time [s]')
            ax.grid(True)

        self.axs[0, 0].set_ylabel('SOC [%]')
        self.axs[0, 1].set_ylabel('Temperature [°C]')
        self.axs[1, 0].set_ylabel('Voltage [V]')
        self.axs[1, 1].set_ylabel('Current [A]')
        self.axs[2, 0].set_ylabel('Residual Capacity [mAh]')

        # Animation functions to update each plot
        self.ani_soc = FuncAnimation(self.fig, self.update_soc_plot, interval=1000)
        self.ani_temp = FuncAnimation(self.fig, self.update_temp_plot, interval=1000)
        self.ani_voltage = FuncAnimation(self.fig, self.update_voltage_plot, interval=1000)
        self.ani_current = FuncAnimation(self.fig, self.update_current_plot, interval=1000)
        self.ani_capacity = FuncAnimation(self.fig, self.update_capacity_plot, interval=1000)

    def format_xaxis(self):
        for ax in self.axs.flat:
            ax.xaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{x:.0f} s'))
            ax.set_xlabel('Time [s]')

    def battery_callback(self, data):
        # Calculate the elapsed time
        elapsed_time = rospy.get_time() - self.start_time

        # Append the elapsed time and each parameter to their respective lists
        self.time_data.append(elapsed_time)
        self.soc_data.append(data.data[0])
        self.temp_data.append(data.data[1])
        self.voltage_data.append(data.data[2])
        self.current_data.append(data.data[3])
        self.capacity_data.append(data.data[4])

    def update_soc_plot(self, frame):
        self.axs[0, 0].clear()
        self.axs[0, 0].plot(self.time_data, self.soc_data, label="SOC")
        self.axs[0, 0].set_title('State of Charge (SOC) [%]')
        self.axs[0, 0].set_ylabel('SOC [%]')
        self.axs[0, 0].legend()
        self.axs[0, 0].grid(True)
        self.format_xaxis()

    def update_temp_plot(self, frame):
        self.axs[0, 1].clear()
        self.axs[0, 1].plot(self.time_data, self.temp_data, label="Temperature")
        self.axs[0, 1].set_title('Temperature [°C]')
        self.axs[0, 1].set_ylabel('Temperature [°C]')
        self.axs[0, 1].legend()
        self.axs[0, 1].grid(True)
        self.format_xaxis()

    def update_voltage_plot(self, frame):
        self.axs[1, 0].clear()
        self.axs[1, 0].plot(self.time_data, self.voltage_data, label="Voltage")
        self.axs[1, 0].set_title('Voltage [V]')
        self.axs[1, 0].set_ylabel('Voltage [V]')
        self.axs[1, 0].legend()
        self.axs[1, 0].grid(True)
        self.format_xaxis()

    def update_current_plot(self, frame):
        self.axs[1, 1].clear()
        self.axs[1, 1].plot(self.time_data, self.current_data, label="Current")
        self.axs[1, 1].set_title('Current [A]')
        self.axs[1, 1].set_ylabel('Current [A]')
        self.axs[1, 1].legend()
        self.axs[1, 1].grid(True)
        self.format_xaxis()

    def update_capacity_plot(self, frame):
        self.axs[2, 0].clear()
        self.axs[2, 0].plot(self.time_data, self.capacity_data, label="Residual Capacity")
        self.axs[2, 0].set_title('Residual Capacity [mAh]')
        self.axs[2, 0].set_ylabel('Residual Capacity [mAh]')
        self.axs[2, 0].legend()
        self.axs[2, 0].grid(True)
        self.format_xaxis()

    def save_data(self):
        # Save the data to a CSV file
        with open(self.save_directory + '/battery_data.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time [s]', 'SOC [%]', 'Temperature [°C]', 'Voltage [V]', 'Current [A]', 'Residual Capacity [mAh]'])
            for i in range(len(self.time_data)):
                writer.writerow([self.time_data[i], self.soc_data[i], self.temp_data[i], self.voltage_data[i], self.current_data[i], self.capacity_data[i]])
        rospy.loginfo("Data saved to battery_data.csv.")

    def save_plot(self):
        # Save the plot to a PNG file
        self.fig.savefig(self.save_directory + '/battery_plot.png')
        rospy.loginfo("Plot saved to battery_plot.png.")

        # Save each subplot separately
        plot_titles = ['SOC', 'Temperature', 'Voltage', 'Current', 'Residual_Capacity']
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
            rospy.loginfo("Shutting down Battery Monitor.")
            self.save_data()  # Save data before shutting down
            self.save_plot()  # Save plot before shutting down
            plt.close(self.fig)
            sys.exit(0)

    def run(self): 
        rate = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('battery_monitor_node', anonymous=True)
    
    # Create an instance of the BatteryMonitor class
    monitor = BatteryMonitor()

    # Set up signal handler for clean shutdown
    def signal_handler(sig, frame):
        rospy.loginfo("Shutting down Battery Monitor due to signal.")
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
