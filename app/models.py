""" Models for the senseRF environment monitoring application"""

import peewee


model_db = peewee.SqliteDatabase("enviro.db")

class BaseModel(peewee.Model):
    class Meta:
        database = model_db

class Sensor(BaseModel):
    """
    Object model for physical sensor information
    """
    unique_id = peewee.IntegerField()
    location = peewee.CharField()
    last_updated = peewee.DateTimeField()
    name = peewee.CharField()
    refresh_rate = peewee.IntegerField()  # Update rate for the sensor.

class SensorData(BaseModel):
    sensor_type = peewee.CharField() # The sensor name the info came from
    sensor = peewee.ForeignKeyField(Sensor) # Link a sensor entry
    sensor_val = peewee.FloatField()  # The value
    updated_at = peewee.DateTimeField()  # The timestamp (time.datetime)

def init_db():
    try:
        Sensor.create_table()
    except:
        print("Sensor table already exists.")

    try:
        SensorData.create_table()
    except:
        print("SensorData table already exists.")
