import cv2
import tkinter as tk
from PIL import Image, ImageTk

class CameraApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Camera Feed in Tkinter")

        # OpenCV Video Capture
        self.cap = cv2.VideoCapture(4)

        # Create a Label Widget to Display Camera Feed
        self.label = tk.Label(root)
        self.label.pack()

        # Update the frame continuously
        self.update_frame()

        # Close the camera when the window is closed
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def update_frame(self):
        # Capture frame from camera
        ret, frame = self.cap.read()
        if ret:
            # Convert BGR to RGB
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)

            # Display image in label
            self.label.imgtk = imgtk
            self.label.configure(image=imgtk)

        # Schedule the next frame update
        self.root.after(10, self.update_frame)

    def on_closing(self):
        self.cap.release()  # Release the camera
        self.root.destroy()  # Destroy the window

if __name__ == "__main__":
    root = tk.Tk()
    app = CameraApp(root)
    root.mainloop()
