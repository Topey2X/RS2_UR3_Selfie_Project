import tkinter as tk

class gui:
    def create_gui(self):
        self.root = tk.Tk()
        self.root.title("RobotSelfie")

        capture_image = tk.PhotoImage(
            file="/home/darren2004/git/RS2_UR3_Selfie_Project/robotSelfie/scripts/icons/capture.png"
        )
        use_image = tk.PhotoImage(
            file="/home/darren2004/git/RS2_UR3_Selfie_Project/robotSelfie/scripts/icons/tick.png"
        )

        self.capture_button = tk.Button(
            self.root,
            compound=tk.BOTTOM,
            image=capture_image,
            text="Capture",
            command=self.capture_image,
            bg="white",
            fg="black",
            font=("Helvetica", 14, "bold"),
            padx=20,
            pady=10,
            relief="raised",
        )
        self.capture_button.pack(pady=10)

        self.use_button = tk.Button(
            self.root,
            compound=tk.BOTTOM,
            image=use_image,
            text="Use Image",
            command=self.use_image,
            state=tk.DISABLED,
            bg="gray",
            fg="white",
            font=("Helvetica", 12),
            padx=15,
            pady=8,
        )
        self.use_button.pack(pady=5)
        
        self.image_label = tk.Label(self.root)
        self.image_label.pack(side=tk.RIGHT, padx=10, pady=10)

        self.root.mainloop()

g = gui()
g.create_gui()