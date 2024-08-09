import sdu_controllers


def test_sdu_controllers():
    assert sdu_controllers.add_one(1) == 2
    assert sdu_controllers.one_plus_one() == 2
