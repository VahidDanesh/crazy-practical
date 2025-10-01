"""

Grid map library in python

author: Atsushi Sakai, some improvements by Vahid Danesh

"""
import matplotlib.pyplot as plt
import numpy as np


class GridMap:
    """
    GridMap class
    """

    def __init__(self, width: int, height: int, resolution: float,
                 center_x: float, center_y: float, init_val: float = 0.0):
        """__init__

        :param width: number of grid for width
        :param height: number of grid for height
        :param resolution: grid resolution [m]
        :param center_x: center x position  [m]
        :param center_y: center y position [m]
        :param init_val: initial value for all grid (float)
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        self.center_x = center_x
        self.center_y = center_y

        self.left_lower_x = self.center_x - self.width / 2.0 * self.resolution
        self.left_lower_y = self.center_y - self.height / 2.0 * self.resolution

        self.n_data = self.width * self.height
        # Use NumPy array for fast operations (store floats directly)
        self.data = np.full((self.height, self.width), init_val, dtype=float)

    def get_value_from_xy_index(self, x_ind: int, y_ind: int) -> float:
        """get_value_from_xy_index

        when the index is out of grid map area, return None

        :param x_ind: x index
        :param y_ind: y index
        """
        if 0 <= x_ind < self.width and 0 <= y_ind < self.height:
            return self.data[y_ind, x_ind]
        else:
            return None

    def get_xy_index_from_xy_pos(self, x_pos: float, y_pos: float) -> tuple:
        """get_xy_index_from_xy_pos

        :param x_pos: x position [m]
        :param y_pos: y position [m]
        :return: tuple of (x_ind, y_ind) or (None, None) if out of bounds
        """
        x_ind = self.calc_xy_index_from_position(
            x_pos, self.left_lower_x, self.width)
        y_ind = self.calc_xy_index_from_position(
            y_pos, self.left_lower_y, self.height)

        return x_ind, y_ind

    def set_value_from_xy_pos(self, x_pos: float, y_pos: float, val: float) -> bool:
        """set_value_from_xy_pos

        return bool flag, which means setting value is succeeded or not

        :param x_pos: x position [m]
        :param y_pos: y position [m]
        :param val: grid value (float)
        """
        x_ind, y_ind = self.get_xy_index_from_xy_pos(x_pos, y_pos)

        if x_ind is None or y_ind is None:
            return False  # NG

        flag = self.set_value_from_xy_index(x_ind, y_ind, val)

        return flag

    def set_value_from_xy_index(self, x_ind: int, y_ind: int, val: float) -> bool:
        """set_value_from_xy_index

        return bool flag, which means setting value is succeeded or not

        :param x_ind: x index
        :param y_ind: y index
        :param val: grid value (float)
        """
        if x_ind is None or y_ind is None:
            return False

        if 0 <= x_ind < self.width and 0 <= y_ind < self.height:
            self.data[y_ind, x_ind] = float(val)
            return True  # OK
        else:
            return False  # NG

    def set_value_from_polygon(self, pol_x, pol_y, val: float, inside: bool = True):
        """set_value_from_polygon

        Setting value inside or outside polygon

        :param pol_x: x position list for a polygon
        :param pol_y: y position list for a polygon
        :param val: grid value (float)
        :param inside: setting data inside or outside
        """
        # making ring polygon
        if (pol_x[0] != pol_x[-1]) or (pol_y[0] != pol_y[-1]):
            pol_x = np.append(pol_x, pol_x[0])
            pol_y = np.append(pol_y, pol_y[0])

        # setting value for all grid
        for x_ind in range(self.width):
            for y_ind in range(self.height):
                x_pos, y_pos = self.calc_grid_central_xy_position_from_xy_index(
                    x_ind, y_ind)

                flag = self.check_inside_polygon(x_pos, y_pos, pol_x, pol_y)

                if flag is inside:
                    self.set_value_from_xy_index(x_ind, y_ind, val)

    def add_obstacle(self, center_x: float, center_y: float, radius: float, 
                     val: float = 1.0, n_sides: int = 16):
        """Add a circular obstacle to the grid map
        
        The circle is approximated by a polygon with n_sides (default 16).
        
        :param center_x: x position of obstacle center [m]
        :param center_y: y position of obstacle center [m]
        :param radius: radius of the obstacle [m]
        :param val: grid value for the obstacle (default 1.0)
        :param n_sides: number of sides for polygon approximation (default 16)
        """
        # Generate polygon vertices approximating a circle
        angles = np.linspace(0, 2 * np.pi, n_sides, endpoint=False)
        pol_x = center_x + radius * np.cos(angles)
        pol_y = center_y + radius * np.sin(angles)
        
        # Set values inside the polygon
        self.set_value_from_polygon(pol_x, pol_y, val, inside=True)

    def calc_grid_index_from_xy_index(self, x_ind: int, y_ind: int) -> int:
        """Calculate flat grid index from x,y indices (for compatibility)"""
        grid_ind = int(y_ind * self.width + x_ind)
        return grid_ind

    def calc_xy_index_from_grid_index(self, grid_ind: int) -> tuple:
        """Calculate x,y indices from flat grid index (for compatibility)"""
        y_ind, x_ind = divmod(grid_ind, self.width)
        return x_ind, y_ind

    def calc_grid_index_from_xy_pos(self, x_pos: float, y_pos: float) -> int:
        """calc_grid_index_from_xy_pos

        :param x_pos: x position [m]
        :param y_pos: y position [m]
        """
        x_ind = self.calc_xy_index_from_position(
            x_pos, self.left_lower_x, self.width)
        y_ind = self.calc_xy_index_from_position(
            y_pos, self.left_lower_y, self.height)

        return self.calc_grid_index_from_xy_index(x_ind, y_ind)

    def calc_grid_central_xy_position_from_grid_index(self, grid_ind: int) -> tuple:
        """Calculate central x,y position from flat grid index"""
        x_ind, y_ind = self.calc_xy_index_from_grid_index(grid_ind)
        return self.calc_grid_central_xy_position_from_xy_index(x_ind, y_ind)

    def calc_grid_central_xy_position_from_xy_index(self, x_ind: int, y_ind: int) -> tuple:
        """Calculate central x,y position from x,y indices"""
        x_pos = self.calc_grid_central_xy_position_from_index(
            x_ind, self.left_lower_x)
        y_pos = self.calc_grid_central_xy_position_from_index(
            y_ind, self.left_lower_y)

        return x_pos, y_pos

    def calc_grid_central_xy_position_from_index(self, index: int, lower_pos: float) -> float:
        """Calculate central position from index"""
        return lower_pos + index * self.resolution + self.resolution / 2.0

    def calc_xy_index_from_position(self, pos: float, lower_pos: float, max_index: int) -> int:
        """Calculate index from position"""
        ind = int(np.floor((pos - lower_pos) / self.resolution))
        if 0 <= ind <= max_index:
            return ind
        else:
            return None

    def check_occupied_from_xy_index(self, x_ind: int, y_ind: int, occupied_val: float) -> bool:
        """Check if grid cell is occupied based on threshold value"""
        val = self.get_value_from_xy_index(x_ind, y_ind)

        if val is None or val >= occupied_val:
            return True
        else:
            return False

    def expand_grid(self, occupied_val: float = 1.0):
        """Expand occupied grid cells to neighboring cells"""
        x_inds, y_inds, values = [], [], []

        for ix in range(self.width):
            for iy in range(self.height):
                if self.check_occupied_from_xy_index(ix, iy, occupied_val):
                    x_inds.append(ix)
                    y_inds.append(iy)
                    values.append(self.get_value_from_xy_index(ix, iy))

        for (ix, iy, value) in zip(x_inds, y_inds, values):
            self.set_value_from_xy_index(ix + 1, iy, val=value)
            self.set_value_from_xy_index(ix, iy + 1, val=value)
            self.set_value_from_xy_index(ix + 1, iy + 1, val=value)
            self.set_value_from_xy_index(ix - 1, iy, val=value)
            self.set_value_from_xy_index(ix, iy - 1, val=value)
            self.set_value_from_xy_index(ix - 1, iy - 1, val=value)

    @staticmethod
    def check_inside_polygon(iox, ioy, x, y):

        n_point = len(x) - 1
        inside = False
        for i1 in range(n_point):
            i2 = (i1 + 1) % (n_point + 1)

            if x[i1] >= x[i2]:
                min_x, max_x = x[i2], x[i1]
            else:
                min_x, max_x = x[i1], x[i2]
            if not min_x <= iox < max_x:
                continue

            tmp1 = (y[i2] - y[i1]) / (x[i2] - x[i1])
            if (y[i1] + tmp1 * (iox - x[i1]) - ioy) > 0.0:
                inside = not inside

        return inside

    def print_grid_map_info(self):
        print("width:", self.width)
        print("height:", self.height)
        print("resolution:", self.resolution)
        print("center_x:", self.center_x)
        print("center_y:", self.center_y)
        print("left_lower_x:", self.left_lower_x)
        print("left_lower_y:", self.left_lower_y)
        print("n_data:", self.n_data)

    def plot_grid_map(self, ax=None, use_world_coords=False):
        """Plot the grid map as a heatmap
        
        :param ax: matplotlib axis to plot on (creates new if None)
        :param use_world_coords: if True, use real world coordinates; if False, use indices
        """
        grid_data = self.data
        if not ax:
            fig, ax = plt.subplots()
        
        if use_world_coords:
            # Calculate extent in world coordinates
            extent = [
                self.left_lower_x,
                self.left_lower_x + self.width * self.resolution,
                self.left_lower_y,
                self.left_lower_y + self.height * self.resolution
            ]
            heat_map = ax.imshow(grid_data, cmap="Blues", vmin=0.0, vmax=1.0,
                                origin='lower', extent=extent)
            ax.set_xlabel('X Position [m]')
            ax.set_ylabel('Y Position [m]')
            plt.axis("equal")
        else:
            heat_map = ax.pcolor(grid_data, cmap="Blues", vmin=0.0, vmax=1.0)
            ax.set_xlabel('X Index')
            ax.set_ylabel('Y Index')
            plt.axis("equal")
        
        ax.set_title('Grid Map')

        return heat_map


def polygon_set_demo():
    """Demo function showing polygon-based grid value setting"""
    ox = [0.0, 4.35, 20.0, 50.0, 100.0, 130.0, 40.0]
    oy = [0.0, -4.15, -20.0, 0.0, 30.0, 60.0, 80.0]

    grid_map = GridMap(600, 290, 0.7, 60.0, 30.5)

    grid_map.set_value_from_polygon(ox, oy, 1.0, inside=False)

    grid_map.plot_grid_map()

    plt.axis("equal")
    plt.grid(True)


def position_set_demo():
    """Demo function showing position-based grid value setting"""
    grid_map = GridMap(100, 120, 0.5, 10.0, -0.5)

    grid_map.set_value_from_xy_pos(10.1, -1.1, 1.0)
    grid_map.set_value_from_xy_pos(10.1, -0.1, 1.0)
    grid_map.set_value_from_xy_pos(10.1, 1.1, 1.0)
    grid_map.set_value_from_xy_pos(11.1, 0.1, 1.0)
    grid_map.set_value_from_xy_pos(10.1, 0.1, 1.0)
    grid_map.set_value_from_xy_pos(9.1, 0.1, 1.0)

    grid_map.plot_grid_map()

    plt.axis("equal")
    plt.grid(True)


def main():
    print("start!!")

    position_set_demo()
    polygon_set_demo()

    plt.show()

    print("done!!")


if __name__ == '__main__':
    main()
