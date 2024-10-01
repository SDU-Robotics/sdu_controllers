*************
API Reference
*************

This page contains the API documentation of sdu_controllers. The following classes are available:

Controllers:

* :ref:`Controller <controller-api>`
* :ref:`AdmittanceControllerPosition <admittance-controller-position-api>`
* :ref:`PDController <pd-controller-api>`

Math:

* :ref:`ForwardDynamics <forward-dynamics-api>`
* :ref:`InverseDynamics <inverse-dynamics-api>`
* :ref:`InverseDynamicsJointSpace <inverse-dynamics-joint-space-api>`

Models:

* :ref:`RobotModel <robot-model-api>`
* :ref:`BreedingBlanketHandlingRobotModel <breeding-blanket-handling-robot-model-api>`
* :ref:`URRobotModel <ur-robot-model-api>`

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

AdmittanceControllerPosition
============================

.. doxygenclass:: sdu_controllers::controllers::AdmittanceControllerPosition
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
