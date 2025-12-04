"""
navigation_display.py

Visual display for rover navigation showing position, heading, and grade.

This module provides a real-time visualization of the rover's position
in the X-Y plane with heading arrow and grade-based color coding.
"""

import tkinter as tk
import math


class NavigationDisplay:
    """
    Visual navigation display showing position, heading, and grade.
    
    Displays the rover's position on a 2D grid with:
    - X-Y coordinate system (positive X right, positive Y up)
    - Arrow indicating heading direction in the X-Y plane
    - Color gradient representing grade (tilt angle):
        - White: 0 degrees (level)
        - Red: 45 degrees
        - Black: 90 degrees (vertical)
    
    Args:
        width (int): Window width in pixels (default: 600)
        height (int): Window height in pixels (default: 600)
        grid_spacing (float): Grid spacing in meters (default: 1.0)
        scale (float): Pixels per meter (default: 50.0)
        title (str): Window title
    """
    
    def __init__(self, **kwargs):
        self.width = kwargs.get("width", 600)
        self.height = kwargs.get("height", 600)
        self.grid_spacing = kwargs.get("grid_spacing", 1.0)
        self.scale = kwargs.get("scale", 50.0)  # pixels per meter
        self.title = kwargs.get("title", "MACRO Navigation Display")
        
        # Position and orientation state
        self.position = (0.0, 0.0)  # (x, y) in meters
        self.heading = 0.0  # heading angle in degrees (0 = positive X)
        self.grade = 0.0  # grade angle in degrees (0 = level, 90 = vertical)
        
        # Center of display (origin)
        self.center_x = self.width // 2
        self.center_y = self.height // 2
        
        # Arrow properties
        self.arrow_length = 30  # pixels
        self.arrow_width = 3
        
        # Initialize tkinter
        self.root = None
        self.canvas = None
        self.rover_arrow = None
        self.position_text = None
        self.grade_text = None
    
    def _grade_to_color(self, grade):
        """
        Convert grade angle to color.
        
        0 degrees = white (#FFFFFF)
        45 degrees = red (#FF0000)
        90 degrees = black (#000000)
        
        Args:
            grade (float): Grade angle in degrees (0-90)
        
        Returns:
            str: Hex color string
        """
        # Clamp grade to 0-90
        grade = max(0, min(90, abs(grade)))
        
        if grade <= 45:
            # White to Red: decrease green and blue
            ratio = grade / 45.0
            r = 255
            g = int(255 * (1 - ratio))
            b = int(255 * (1 - ratio))
        else:
            # Red to Black: decrease red
            ratio = (grade - 45) / 45.0
            r = int(255 * (1 - ratio))
            g = 0
            b = 0
        
        return f"#{r:02x}{g:02x}{b:02x}"
    
    def _world_to_screen(self, x, y):
        """
        Convert world coordinates (meters) to screen coordinates (pixels).
        
        Positive X is to the right, positive Y is up.
        
        Args:
            x (float): X position in meters
            y (float): Y position in meters
        
        Returns:
            tuple: (screen_x, screen_y) in pixels
        """
        screen_x = self.center_x + x * self.scale
        screen_y = self.center_y - y * self.scale  # Y is inverted in screen coords
        return screen_x, screen_y
    
    def _draw_grid(self):
        """Draw the coordinate grid with axes."""
        # Grid color
        grid_color = "#D3D3D3"  # Light grey
        axis_color = "#808080"  # Darker grey for axes
        
        # Calculate grid range
        x_range = self.width / (2 * self.scale)
        y_range = self.height / (2 * self.scale)
        
        # Draw vertical grid lines
        x = 0
        while x <= x_range:
            for sign in [1, -1]:
                sx, _ = self._world_to_screen(sign * x, 0)
                if 0 <= sx <= self.width:
                    color = axis_color if x == 0 else grid_color
                    width = 2 if x == 0 else 1
                    self.canvas.create_line(sx, 0, sx, self.height, 
                                          fill=color, width=width)
            x += self.grid_spacing
        
        # Draw horizontal grid lines
        y = 0
        while y <= y_range:
            for sign in [1, -1]:
                _, sy = self._world_to_screen(0, sign * y)
                if 0 <= sy <= self.height:
                    color = axis_color if y == 0 else grid_color
                    width = 2 if y == 0 else 1
                    self.canvas.create_line(0, sy, self.width, sy,
                                          fill=color, width=width)
            y += self.grid_spacing
        
        # Draw axis labels
        label_offset = 15
        # X-axis label (positive direction)
        self.canvas.create_text(self.width - label_offset, self.center_y - label_offset,
                               text="+X", fill=axis_color, font=("Arial", 10, "bold"))
        # Y-axis label (positive direction)
        self.canvas.create_text(self.center_x + label_offset, label_offset,
                               text="+Y", fill=axis_color, font=("Arial", 10, "bold"))
        
        # Origin label
        self.canvas.create_text(self.center_x + 15, self.center_y + 15,
                               text="(0,0)", fill=axis_color, font=("Arial", 8))
    
    def _draw_rover(self):
        """Draw the rover arrow at current position with heading and grade color."""
        # Get screen position
        sx, sy = self._world_to_screen(self.position[0], self.position[1])
        
        # Calculate arrow endpoint based on heading
        heading_rad = math.radians(self.heading)
        end_x = sx + self.arrow_length * math.cos(heading_rad)
        end_y = sy - self.arrow_length * math.sin(heading_rad)  # Y inverted
        
        # Get color based on grade
        color = self._grade_to_color(self.grade)
        
        # Draw arrow
        if self.rover_arrow:
            self.canvas.delete(self.rover_arrow)
        
        self.rover_arrow = self.canvas.create_line(
            sx, sy, end_x, end_y,
            fill=color, width=self.arrow_width,
            arrow=tk.LAST, arrowshape=(12, 15, 5)
        )
        
        # Draw position circle at base
        radius = 5
        self.canvas.create_oval(
            sx - radius, sy - radius,
            sx + radius, sy + radius,
            fill=color, outline=color
        )
    
    def _draw_info(self):
        """Draw position and grade information."""
        # Position text
        pos_text = f"Position: ({self.position[0]:.2f}, {self.position[1]:.2f}) m"
        if self.position_text:
            self.canvas.delete(self.position_text)
        self.position_text = self.canvas.create_text(
            10, 10, anchor=tk.NW,
            text=pos_text, fill="black", font=("Arial", 10)
        )
        
        # Heading text
        heading_text = f"Heading: {self.heading:.1f}°"
        self.canvas.create_text(
            10, 28, anchor=tk.NW,
            text=heading_text, fill="black", font=("Arial", 10)
        )
        
        # Grade text with color indicator
        grade_text = f"Grade: {self.grade:.1f}°"
        color = self._grade_to_color(self.grade)
        if self.grade_text:
            self.canvas.delete(self.grade_text)
        self.grade_text = self.canvas.create_text(
            10, 46, anchor=tk.NW,
            text=grade_text, fill=color, font=("Arial", 10, "bold")
        )
    
    def update(self, **kwargs):
        """
        Update the display with new position, heading, and grade.
        
        Args:
            position (tuple): (x, y) position in meters
            heading (float): Heading angle in degrees (0 = positive X direction)
            grade (float): Grade angle in degrees (0 = level)
        """
        if "position" in kwargs:
            self.position = kwargs["position"]
        if "heading" in kwargs:
            self.heading = kwargs["heading"]
        if "grade" in kwargs:
            self.grade = kwargs["grade"]
        
        if self.canvas:
            self._refresh()
    
    def _refresh(self):
        """Refresh the entire display."""
        self.canvas.delete("all")
        self._draw_grid()
        self._draw_rover()
        self._draw_info()
    
    def run(self):
        """Start the display window (blocking)."""
        self.root = tk.Tk()
        self.root.title(self.title)
        self.root.resizable(False, False)
        
        self.canvas = tk.Canvas(
            self.root,
            width=self.width,
            height=self.height,
            bg="white"
        )
        self.canvas.pack()
        
        self._refresh()
        self.root.mainloop()
    
    def start(self):
        """Initialize the display window (non-blocking setup)."""
        self.root = tk.Tk()
        self.root.title(self.title)
        self.root.resizable(False, False)
        
        self.canvas = tk.Canvas(
            self.root,
            width=self.width,
            height=self.height,
            bg="white"
        )
        self.canvas.pack()
        
        self._refresh()
    
    def process_events(self):
        """Process pending GUI events (call in main loop)."""
        if self.root:
            self.root.update()
    
    def close(self):
        """Close the display window."""
        if self.root:
            self.root.destroy()
            self.root = None


if __name__ == "__main__":
    # Demo: show display with sample positions
    import time
    
    display = NavigationDisplay(
        width=600,
        height=600,
        grid_spacing=1.0,
        scale=50.0
    )
    
    display.start()
    
    # Animate a circular path
    try:
        t = 0
        while True:
            # Circular motion
            x = 3 * math.cos(t)
            y = 3 * math.sin(t)
            heading = math.degrees(t) + 90  # Tangent to circle
            grade = 20 * math.sin(t * 2)  # Oscillating grade
            
            display.update(
                position=(x, y),
                heading=heading,
                grade=abs(grade)
            )
            display.process_events()
            
            t += 0.05
            time.sleep(0.05)
    except tk.TclError:
        # Window was closed
        pass
    except KeyboardInterrupt:
        display.close()
