from datetime import datetime
import peewee
import itertools
import random
from time import sleep


from models import Sensor, SensorData, init_db

sensors = [{"id": 1,
            "location": "Living Room",
            "late_updated": datetime.now(),
            "name": "greenpi",
            "refresh_rate": 30}
          ]

class SensorNode:
    types = {"temperature", "humidity", "light"}
    new_id = next(itertools.count())
    def __init__(self, name, location, refresh_rate=30):
        self.name = name
        self.location = location
        self.refresh_rate = refresh_rate
        self.id = SensorNode.new_unique_id()
        self.last_updated = 0
        self.record = Sensor.create(unique_id=self.id, location=self.location,
                                    last_updated=self.last_updated, name=self.name,
                                    refresh_rate=self.refresh_rate)
        self.record.save()

    def update_values(self, sensors_to_update=[]):
        def cast_arg(arg):
            if type(arg) is str:
                if arg == "all":
                    return SensorNode.types
                else:
                    return {arg} & SensorNode.types
            else:
                return set(arg) & SensorNode.types

        for item in cast_arg(sensors_to_update):
            self._update(item)

    def _update(self, sensor_type):
        if sensor_type is "temperature":
            value = self.get_rand_temperature()
        elif sensor_type is "humidity":
            value = self.get_rand_humidity()
        elif sensor_type is "light":
            value = self.get_rand_light()
        updated_at = datetime.now()
        data = {"sensor_type": sensor_type, "sensor": self.record,
                "sensor_val": value, "updated_at": datetime.now()}
        record = SensorData(**data)
        record.save()

    @staticmethod
    def get_rand_temperature():
        return random.uniform(62.0, 64.0)

    @staticmethod
    def get_rand_humidity():
        return random.uniform(37.0, 40.0)

    @staticmethod
    def get_rand_light():
        return random.uniform(300.0, 305.0)

    @classmethod
    def new_unique_id(cls):
        return cls.new_id


if __name__ == "__main__":
    init_db()
    sensor = SensorNode("greenpi", "Living Room")
    for number in range(10):
        sensor.update_values("all")
        sleep(.01)
    