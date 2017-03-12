drop table if exists sensors;
create table sensor (
  id integer primary key,
  name text not null,
  created_at timestamp not null,
  updated_at timestamp not null
);

drop table if exists sensor_data_points;
create table sensor_data_points (
  id integer primary key,
  sensor_id integer not null,
  sensor_name text not null,
  sensor_value real not null,
  created_at timestamp not null,
  updated_at timestamp not null
);

drop table if exists tokens;
create table tokens (
  id integer primary key,
  token text not null,
  created_at timestamp not null,
  updated_at timestamp not null
);