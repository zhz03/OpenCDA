# -*- coding: utf-8 -*-
# Author: Runsheng Xu <rxx3386@ucla.edu>, Zhaoliang Zheng <zhz03@g.ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
import os

import carla

import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import load_yaml


def run_scenario(opt, config_yaml):
    try:
        scenario_params = load_yaml(config_yaml)

        # basic_config
        basic_config = scenario_params['basic_config']

        # create CAV world
        cav_world = CavWorld(opt.apply_ml)

        if basic_config['xodr_path'] != None:
            current_path = os.path.dirname(os.path.realpath(__file__))
            xodr_path = os.path.join(
                current_path,
                basic_config['xodr_path'])
        else:
            xodr_path = basic_config['xodr_path']

        # create scenario manager
        # This is where we need to make it more general: xodr_path=None
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   xodr_path=xodr_path,
                                                   town=xodr_path,
                                                   cav_world=cav_world)

        if opt.record:
            # This is where we need to make it more general: "over_take.log"
            scenario_manager.client. \
                start_recorder(basic_config['log_name'], True)

        # This is where we need to make it more general,map_helper=None
        single_cav_list = \
            scenario_manager.create_vehicle_manager(application=['single'],
                                                    map_helper=None)

        # create background traffic in carla
        traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='single_2lanefree_carla',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
        # run steps
        while True:
            scenario_manager.tick()
            transform = single_cav_list[0].vehicle.get_transform()
            # This is where we need to make it more general
            spectator.set_transform(carla.Transform(
                transform.location +
                carla.Location(
                    z=50),
                carla.Rotation(
                    pitch=-
                    90)))

            for i, single_cav in enumerate(single_cav_list):
                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)

    finally:
        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()

        for v in single_cav_list:
            v.destroy()
        for v in bg_veh_list:
            v.destroy()

