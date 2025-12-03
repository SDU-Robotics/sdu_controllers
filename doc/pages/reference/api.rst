*************
API Reference
*************

This page contains the API documentation of sdu_controllers. The following classes are available:

Controllers:

* :ref:`Controller <controller-api>`
* :ref:`PDController <pd-controller-api>`
* :ref:`AdmittanceControllerCartesian <admittance-controller-position-api>`
* :ref:`OperationalSpaceController <operational-space-controller-api>`

Math:

* :ref:`ForwardDynamics <forward-dynamics-api>`
* :ref:`InverseDynamics <inverse-dynamics-api>`
* :ref:`InverseDynamicsJointSpace <inverse-dynamics-joint-space-api>`

Models:

* :ref:`RobotModel <robot-model-api>`
* :ref:`BreedingBlanketHandlingRobotModel <breeding-blanket-handling-robot-model-api>`
* :ref:`URRobotModel <ur-robot-model-api>`

HAL:

* :ref:`Robot <robot-api>`
* :ref:`URRobot <ur-robot-api>`

.. _controller-api:

Controller
==========

.. doxygenclass:: sdu_controllers::controllers::Controller
    :project: sdu_controllers
    :members:

.. _pd-controller-api:

PDController
============

.. doxygenclass:: sdu_controllers::controllers::PDController
    :project: sdu_controllers
    :members:


.. _admittance-controller-position-api:

AdmittanceControllerCartesian
============================

.. doxygenclass:: sdu_controllers::controllers::AdmittanceControllerCartesian
    :project: sdu_controllers
    :members:

.. _operational-space-controller-api:

OperationalSpaceController
==========================

.. doxygenclass:: sdu_controllers::controllers::OperationalSpaceController
    :project: sdu_controllers
    :members:

.. _forward-dynamics-api:

ForwardDynamics
===============

.. doxygenclass:: sdu_controllers::math::ForwardDynamics
    :project: sdu_controllers
    :members:

.. _inverse-dynamics-api:

InverseDynamics
===============

.. doxygenclass:: sdu_controllers::math::InverseDynamics
    :project: sdu_controllers
    :members:

.. _inverse-dynamics-joint-space-api:

InverseDynamicsJointSpace
=========================

.. doxygenclass:: sdu_controllers::math::InverseDynamicsJointSpace
    :project: sdu_controllers
    :members:


.. _robot-model-api:

RobotModel
==========

.. doxygenclass:: sdu_controllers::models::RobotModel
    :project: sdu_controllers
    :members:

.. _ur-robot-model-api:

URRobotModel
============

.. doxygenclass:: sdu_controllers::models::URRobotModel
    :project: sdu_controllers
    :members:

.. _breeding-blanket-handling-robot-model-api:

BreedingBlanketHandlingRobotModel
=================================

.. doxygenclass:: sdu_controllers::models::BreedingBlanketHandlingRobotModel
    :project: sdu_controllers
    :members:

.. _robot-api:

Robot
=====

.. doxygenclass:: sdu_controllers::hal::Robot
    :project: sdu_controllers
    :members:

.. _ur-robot-api:

URRobot
=======

.. doxygenclass:: sdu_controllers::hal::URRobot
    :project: sdu_controllers
    :members:

