#!/usr/bin/python3

from abc import ABC, abstractmethod
from typing import Tuple, List
import os

from math import sqrt, ceil, pi, cos, sin, hypot
import numpy as np

import roslib
from rospkg import RosPack


roslib.load_manifest('world_generator')
ROSPACK_INST = RosPack()


class PointsGenerator(ABC):
    pass


class DiscPoissonGenerator(PointsGenerator):
    def __init__(self, width: float, height: float, min_dist: float, points_number: int) -> None:
        self.__dimensions = (height, width)
        self.__min_dist = min_dist
        self.__points_number = points_number
        self.__attempts = 1000


        self.__cell_size = self.__min_dist/sqrt(2)
        self.__grid = np.full(list(map(lambda x: ceil(x/self.__cell_size), self.__dimensions)), None, dtype=object)

    def grid(self): return self.__grid

    def __iter__(self):
        return self

    def __next__(self):
        if self.__points_number == 0:
            raise StopIteration

        for _ in range(self.__attempts):
            new_point = self.__generate_random_point_around()
            if self.__in_rectangle(new_point) and not self.__in_neighbourhood(new_point):
                x, y = self.__image_to_grid(new_point)
                
                self.__grid[y, x] = new_point
                self.__points_number -= 1
                return new_point
        return None

    def __image_to_grid(self, point: Tuple[float, float]) -> Tuple[int, int]:
        x, y = point
        return tuple(map(int, [x/self.__cell_size, y/self.__cell_size]))

    def __in_rectangle(self, point: Tuple[float, float]) -> bool:
        x, y = point
        return x >= 0 and x <= self.__dimensions[1] and y >= 0 and y <= self.__dimensions[0]

    def __squares_around_point(self, point: Tuple[int, int]):
        gheight, gwidth = self.__grid.shape

        for x in range(max(point[0]-2, 0), min(point[0]+3, gwidth)):
            for y in range(max(point[1]-2, 0), min(point[1]+3, gheight)):
                yield self.__grid[y, x]

    def __in_neighbourhood(self, point: Tuple[float, float]) -> bool:
        grid_point = self.__image_to_grid(point)
        for cell in self.__squares_around_point(grid_point):
            if cell is None: continue

            if hypot(cell[0]-point[0], cell[1]-point[1]) < self.__min_dist: return True
        
        return False          

    def __generate_random_point_around(self) -> Tuple[float, float]:
        return (np.random.uniform(0, self.__dimensions[1]),
                np.random.uniform(0, self.__dimensions[0]))


def main() -> None:
    generated_xml_dronepoints = ''
    models_path = os.path.join(ROSPACK_INST.get_path('world_generator'), 'models/dronepoint')
    models = list(filter(lambda x: os.path.isdir(os.path.join(models_path, x)) and not x.startswith('.'), os.listdir(models_path)))
    
    dronepoint_template_path = os.path.join(ROSPACK_INST.get_path('world_generator'), 'templates', 'dronepoint.world.template')
    with open(dronepoint_template_path, 'r') as file:
        dronepoint_template = file.read()

    print('Generated dronepoints:')
    for point in DiscPoissonGenerator(7, 7, 2, 0):
        if point is None: continue

        x, y = list(map(lambda x: round(x + 1, 2), point))
        model = models[np.random.randint(0, len(models))]
        name = 'dronepoint_' + str(x).replace('.', '-') + '_' + str(y).replace('.', '-')

        print(f'\tmodel={model}; x={x}; y={y}')

        tmp_xml = dronepoint_template.replace("$NAME", name)
        tmp_xml = tmp_xml.replace("$MODEL", model)
        tmp_xml = tmp_xml.replace("$X", str(x))
        tmp_xml = tmp_xml.replace("$Y", str(y))
        generated_xml_dronepoints += tmp_xml

    template_path = os.path.join(ROSPACK_INST.get_path('world_generator'), 'templates', 'default.world.template')

    print('Creating file:')
    models_path = os.path.join(ROSPACK_INST.get_path('clover_simulation'), 'worlds')
    if not os.path.exists(models_path):
        os.makedirs(models_path)
    
    filename = os.path.join(ROSPACK_INST.get_path('clover_simulation'), 'resources', 'worlds', 'clover_aruco.world')
    print('\t' + filename)

    output_file = open(filename, 'w')
    with open(template_path, 'r') as d_file:
        for line in d_file:
            line = line.replace('$DRONEPOINTS_PLACEHOLDER', generated_xml_dronepoints)
            output_file.write(line)

    print('Clover world created successfully')


if __name__ == '__main__':
    main()
