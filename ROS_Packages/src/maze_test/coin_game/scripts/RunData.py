#!/usr/bin/env python

__author__ = 'matt'


class RunData():

    def __init__(self):
        self.__trust_results_for_command = []
        self.__disparity_for_command = []
        self.__health_at_command = []
        self.__delay_for_command = []
        self.__net_trust_at_command = []
        self.__collected_coins_at_command = []
        self.__num_instances = 0

    def add_result(self, **kwargs):

        self.__trust_results_for_command.append(kwargs.get("trust_result"))
        self.__disparity_for_command.append(kwargs.get("disparity"))
        self.__health_at_command.append(kwargs.get("health"))
        self.__delay_for_command.append(kwargs.get("delay"))
        self.__net_trust_at_command.append(kwargs.get("percent_trust"))
        self.__collected_coins_at_command.append(kwargs.get("collected_coins"))
        self.__num_instances += 1

    def get_entry_at_index(self, i):

        toReturn = {}

        if i <= len(self.__trust_results_for_command):
            toReturn.update({"trust_result": self.__trust_results_for_command[i]})

        if i < len(self.__disparity_for_command):
            toReturn.update({"disparity": self.__disparity_for_command[i]})

        if i < len(self.__health_at_command):
            toReturn.update({"health": self.__health_at_command[i]})

        if i < len(self.__delay_for_command):
            toReturn.update({"delay": self.__delay_for_command[i]})

        if i < len(self.__net_trust_at_command):
            toReturn.update({"percent_trust": self.__net_trust_at_command[i]})

        if i < len(self.__collected_coins_at_command):
            toReturn.update({"collected_coins": self.__collected_coins_at_command[i]})

        return toReturn

    def get_num_instances(self):
        return self.__num_instances
