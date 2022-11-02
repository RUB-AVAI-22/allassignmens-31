# BY Lars Buck
# 26.10.2022
import tkinter as tk
import threading
import numpy as np
import rclpy
from PIL import ImageTk
from PIL import Image as IMG
from cv_bridge import CvBridge
from rclpy.node import Node
from std_msgs.msg import String, Int8, Bool
from sensor_msgs.msg import Image


class User(Node):
    # ImageSubscriber Node
    # Holds an empty Image in case there is no image data in the beginning
    image_data = np.ones(shape=[480, 640, 3], dtype=np.uint8)

    # mode lets user choose between 3 image modes ( normal rgb, grayscale, contour/canny image)
    # change mode by typing the mode name into the EditBox ('normal','grayscale','contour')
    mode = "normal"

    def __init__(self):
        super().__init__('image_subscriber')

        self.processed_image_subscriber = self.create_subscription(Image, 'processed_image', self.show_data, 10)
        self.cv_bridge = CvBridge()
        self.br = CvBridge()

    def show_data(self, data):
        # Display the message on the console
        self.get_logger().info('Receiving processed image data')

        # Convert ROS Image message to OpenCV image
        self.image_data = self.br.imgmsg_to_cv2(data)
        print(self.image_data)


rclpy.init()

node = Node("talker")

mode_pub = node.create_publisher(String, "user_input", 1)
freq_pub = node.create_publisher(Int8, "user_input_freq", 5)
im_rn_pub = node.create_publisher(Bool, "user_input_im_rn", 1)


### GUI PART ###

def get_entry():
    msg = String()
    msg.data = EditBox.get()
    mode_pub.publish(msg)


def get_slider():
    freq_msg = Int8()
    value = slider1.get()
    freq_msg.data = value
    freq_pub.publish(freq_msg)


def request_image_rn():
    im_rn_msg = Bool()
    im_rn_msg.data = True
    print("sending image")
    im_rn_pub.publish(im_rn_msg)


def end():
    node.destroy_node()
    rclpy.shutdown()
    root.destroy()
    print("end")


root = tk.Tk()
root.title(u"Publisher")
root.geometry("800x800")

#
button1 = tk.Button()
button1["text"] = "publish"
button1["command"] = get_entry
button1.pack()

#
EditBox = tk.Entry(width=50)
EditBox.insert(tk.END, "normal")
EditBox.pack()

#
button2 = tk.Button()
button2["text"] = "End"
button2["command"] = end
button2.pack()

image_subscriber = User()

img = ImageTk.PhotoImage(IMG.fromarray(image_subscriber.image_data * 66))
panel = tk.Label(root, image=img)

panel.pack()

slider1 = tk.Scale(root, from_=1, to=15, tickinterval=.2, length=500, orient=tk.HORIZONTAL, )
slider1.set(1)
slider1.pack()

button3 = tk.Button()
button3["text"] = "set frequency"
button3["command"] = get_slider
button3.pack()

button4 = tk.Button()
button4["text"] = "Image NOW!"
button4["command"] = request_image_rn
button4.pack()


def update_img():
    print("...updating")
    img2 = ImageTk.PhotoImage(IMG.fromarray(image_subscriber.image_data))
    panel.configure(image=img2)
    panel.img = img2
    panel.after(500, update_img)


def spin_sub():
    rclpy.spin(image_subscriber)


t1 = threading.Thread(target=spin_sub)
t1.start()
update_img()
root.mainloop()
