from os import path

import mujoco
import numpy as np
import pytest
from numpy.testing import assert_array_almost_equal, assert_array_equal

import deformable_gym.helpers.mj_utils as mju

XML_STRING = """
<mujoco>
	<worldbody>
		<light diffuse=".7 .7 .7" dir="0 0 -1" directional="true" />
		<geom name="floor" type="plane" size="10 10 10" />
		<body name="cube" pos="0 0 .5">
			<joint name="slider" type="slide" axis="1 0 0"/>
			<geom type="box" size=".5 .5 .5" rgba=" .5 0 0 1" />
			<body name="inner_body"/>
		</body>
		<geom name="trash_geom" type="sphere" size=".5" pos="1 0 .5" rgba="0 .5 0 1"/>
		<body name="trash_body" pos = "0 0 2"/>
	</worldbody>
	<actuator>
		<position name="cube_ctrl" joint="slider" kp="500"/>
	</actuator>
	<sensor>
		<framepos name="cube_pos" objtype="body" objname="cube" />
	</sensor>
</mujoco>
"""
FULL_PATH = path.join(path.dirname(__file__), "test.xml")


@pytest.fixture
def model():
    return mujoco.MjModel.from_xml_path(FULL_PATH)


@pytest.fixture
def data(model):
    return mujoco.MjData(model)


@pytest.mark.parametrize("path", [FULL_PATH])
def test_load_model_from_xml_file(path):
    model, _ = mju.load_model_from_file(path)
    assert model.nq == 1
    assert model.nv == 1
    assert model.nbody == 4
    assert model.njnt == 1
    assert model.ngeom == 3
    assert model.nsensor == 1
    assert model.nu == 1


@pytest.mark.parametrize("xml_string", [XML_STRING])
def test_load_model_from_xml_string(xml_string):
    model, _ = mju.load_model_from_string(xml_string)
    assert model.nq == 1
    assert model.nv == 1
    assert model.nbody == 4
    assert model.njnt == 1
    assert model.ngeom == 3
    assert model.nsensor == 1
    assert model.nu == 1


def test_get_body_names(model):
    names = mju.get_body_names(model)
    assert names == ["cube", "trash_body"]


def test_get_geom_names(model):
    names = mju.get_geom_names(model)
    assert names == ["floor", "trash_geom"]


def test_get_sensor_names(model):
    names = mju.get_sensor_names(model)
    assert names == ["cube_pos"]


@pytest.mark.parametrize("name", ["cube_pos"])
def test_get_sensor_data(model, data, name):
    mujoco.mj_forward(model, data)
    init_sensor_data = mju.get_sensor_data(model, data, name)
    assert_array_equal(init_sensor_data, [0, 0, 0.5])
    data.ctrl[:] = 1
    for _ in range(1000):
        mujoco.mj_step(model, data)
    sensor_data = mju.get_sensor_data(model, data, name)
    assert_array_almost_equal(sensor_data, [0.84335797, 0.0, 0.5], decimal=3)


@pytest.mark.parametrize("name", ["cube"])
def test_get_body_pos(model, data, name):
    mujoco.mj_forward(model, data)
    body_pos = mju.get_body_pos(model, data, name)
    assert_array_equal(body_pos, [0, 0, 0.5])


@pytest.mark.parametrize("name", ["trash_body"])
def test_remove_body(model, data, name):
    mujoco.mj_forward(model, data)
    init_body_pos = data.body(name).xpos
    assert_array_equal(init_body_pos, [0, 0, 2])
    mju.remove_body(model, data, name)
    body_pos = data.body(name).xpos
    assert_array_equal(body_pos, [0, 0, -1000])


@pytest.mark.parametrize("name", ["trash_geom"])
def test_remove_geom(model, data, name):
    mujoco.mj_forward(model, data)
    init_geom_pos = data.geom(name).xpos
    assert_array_equal(init_geom_pos, [0, 2, 0.5])
    mju.remove_geom(model, data, name)
    geom_pos = data.geom(name).xpos
    assert_array_equal(geom_pos, [0, 0, -1000])


@pytest.mark.parametrize("name", ["cube"])
def test_get_body_com(model, data, name):
    mujoco.mj_forward(model, data)
    init_com = data.body(name).xipos
    assert_array_equal(init_com, [0, 0, 0.5])
    data.ctrl[:] = 1
    for _ in range(1000):
        mujoco.mj_step(model, data)
    body_com = data.body(name).xipos
    assert_array_almost_equal(body_com, [0.84335797, 0.0, 0.5], decimal=3)


@pytest.mark.parametrize(
    "id, type, result",
    [
        (1, "body", "cube"),
        (0, "geom", "floor"),
        (0, "joint", "slider"),
        (0, "sensor", "cube_pos"),
        (0, "actuator", "cube_ctrl"),
    ],
)
def test_id2name(model, id, type, result):
    assert mju.id2name(model, id, type) == result


@pytest.mark.parametrize(
    "name, type, result",
    [
        ("cube", "body", 1),
        ("floor", "geom", 0),
        ("slider", "joint", 0),
        ("cube_pos", "sensor", 0),
        ("cube_ctrl", "actuator", 0),
    ],
)
def test_name2id(model, name, type, result):
    assert mju.name2id(model, name, type) == result


@pytest.mark.parametrize(
    "euler, result",
    [
        ([0, 0, 0], [1, 0, 0, 0]),
        ([np.pi / 2, 0, 0], [0.70710678, 0.70710678, 0, 0]),
        ([0, np.pi / 2, 0], [0.70710678, 0, 0.70710678, 0]),
        ([0, 0, np.pi / 2], [0.70710678, 0, 0, 0.70710678]),
        ([np.pi, 0, 0], [0, 1, 0, 0]),
        ([0, np.pi, 0], [0, 0, 1, 0]),
        ([0, 0, np.pi], [0, 0, 0, 1]),
    ],
)
def test_euler2quat(euler, result):
    assert_array_almost_equal(mju.euler2quat(euler), result, decimal=5)
