*************
API Reference
*************

This page contains the API documentation of sdu_controllers. The following classes are available:

Controllers:

* :ref:`Controller <controller-api>`
* :ref:`AdmittanceControllerPosition <admittance-controller-position-api>`

Math:

* :ref:`InverseDynamics <inverse-dynamics-api>`
* :ref:`InverseDynamicsJointSpace <inverse-dynamics-joint-space-api>`

Models:

* :ref:`RobotModel <inverse-dynamics-api>`
* :ref:`BreedingBlanketHandlingRobotModel <inverse-dynamics-joint-space-api>`

.. _controller-api:

Controller
==========

.. doxygenclass:: sdu_controllers::controllers::Controller
    :project: sdu_controllers
    :members:


.. _admittance-controller-position-api:

AdmittanceControllerPosition
============================

.. doxygenclass:: sdu_controllers::controllers::AdmittanceControllerPosition
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


.. _robot_model-api:

RobotModel
==========

.. doxygenclass:: sdu_controllers::models::RobotModel
    :project: sdu_controllers
    :members:

.. _breeding_blanket_handling_robot_model-api:

BreedingBlanketHandlingRobotModel
=================================

.. doxygenclass:: sdu_controllers::models::BreedingBlanketHandlingRobotModel
    :project: sdu_controllers
    :members:
