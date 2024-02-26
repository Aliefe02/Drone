import tkinter as tk
import tkinter.ttk as ttk
import serial
import json
#ser = serial.Serial('COM11', 9600)
class DrawingApp:
    def __init__(self, master):
        self.master = master
        self.master.title("Drawing App")

        # Parameters on the right
        self.param_frame = tk.Frame(master, padx=10,bg='black')
        self.param_frame.pack(side=tk.RIGHT, fill=tk.Y)

        # Create labels and textboxes for parameters
        self.param_entries = []

        label = tk.Label(self.param_frame, text='Height')
        label.pack(pady=2)
        entry = ttk.Entry(self.param_frame)
        entry.insert(0, 2)
        entry.pack(pady=5)
        self.param_entries.append(entry)


        label = tk.Label(self.param_frame, text='Cell Length')
        label.pack(pady=2)
        entry = ttk.Entry(self.param_frame)
        entry.insert(0, 1)
        entry.pack(pady=5)
        self.param_entries.append(entry)

        # Buttons to trigger actions
        self.draw_button = ttk.Button(self.param_frame, text="Start Drone", command=self.draw)
        self.draw_button.pack(pady=5)


        self.reset_button = ttk.Button(self.param_frame, text="Reset", command=self.reset_drawings)
        self.reset_button.pack(pady=5)

        # Canvas on the left
        self.canvas = tk.Canvas(master, bg="black")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Initialize grid size
        self.grid_size = 20

        # Drawn squares and initial grid
        self.drawn_squares = []
        self.master.after(100, self.init_grid)  # Schedule init_grid after a short delay

        # Event bindings for drawing
        self.canvas.bind("<B1-Motion>", self.draw_on_canvas)
        self.canvas.bind("<ButtonRelease-1>", self.release_button)

        # Bind resizing event
        self.master.bind("<Configure>", self.redraw_grid)

    def init_grid(self):
        for i in range(0, self.canvas.winfo_width(), max(1, self.canvas.winfo_width() // self.grid_size)):
            self.canvas.create_line(i, 0, i, self.canvas.winfo_height(), fill="gray", dash=(2, 2), tags="grid")
            self.canvas.create_line(0, i, self.canvas.winfo_width(), i, fill="gray", dash=(2, 2), tags="grid")

    def draw_on_canvas(self, event):
        x, y = event.x, event.y
        square_size = self.canvas.winfo_width() // self.grid_size
        col = x // square_size
        row = y // square_size

        square_coords = (col, row)
        if self.drawn_squares == [] or  self.drawn_squares[-1] != square_coords:
            self.drawn_squares.append(square_coords)
            #print(square_coords)

        # Draw on the canvas
        x1 = col * square_size
        y1 = row * square_size
        x2 = x1 + square_size
        y2 = y1 + square_size

        self.canvas.create_rectangle(x1, y1, x2, y2, fill="white")

    def release_button(self, event):
        pass  # Add any cleanup or additional actions here

    def draw(self):
        if self.drawn_squares == []:    # If no path drawn, dont send
            return
        height = self.param_entries[0].get()
        cell_length = self.param_entries[1].get()
        # Call send_path_to_drone with the parameters and drawn squares
        send_path_to_drone(self.drawn_squares, height, cell_length)

    def redraw_grid(self, event=None):
        self.canvas.delete("grid")
        for i in range(0, self.canvas.winfo_width(), max(1, self.canvas.winfo_width() // self.grid_size)):
            self.canvas.create_line(i, 0, i, self.canvas.winfo_height(), fill="gray", dash=(2, 2), tags="grid")
            self.canvas.create_line(0, i, self.canvas.winfo_width(), i, fill="gray", dash=(2, 2), tags="grid")

    def reset_drawings(self):
        # Clear drawn squares and the canvas
        self.drawn_squares.clear()
        self.canvas.delete("all")
        # Redraw the grid after resetting
        self.redraw_grid()

def send_path_to_drone(path, height, cell_length):
    data = {"path":path,"height":height,"cell_length":cell_length,"check":"correct"}
    data_string = json.dumps(data)
    print(data_string)
    #ser.write(data_string.encode())
    #ser.write(b'\n')


def main():
    
    root = tk.Tk()
    root.geometry("1200x600")
    app = DrawingApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
