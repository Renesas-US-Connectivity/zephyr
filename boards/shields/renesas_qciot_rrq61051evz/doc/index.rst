.. _renesas_qciot_rrq61051evz_shield:

Renesas RRQ61051EVZ Board
#########################

Overview
********

The Renesas QCIOT RRQ61051EVZ carries a `RRQ61051EVZ`_ Wi-Fi 6 module
on a board that provides both `Digilent Pmod`_ |trade| and
`mikroBUS`_ |trade| interfaces.

.. figure:: qciot-rrq61051evz.webp
   :align: center
   :alt: Renesas QCIOT RRQ61051EVZ

   Renesas QCIOT RRQ61051EVZ (Credit: Renesas Electronics)

Requirements
************

This shield can only be used with a board that provides a Pmod |trade|
socket and defines the ``pmod_serial`` node label or one that provides 
a mikroBUS |trade| socket and defines the ``mikrobus_spi` node label
(see :ref:`shields` for more details).

Programming
***********

Set ``--shield <shield designator>`` when you invoke ``west build``. For
example:

.. zephyr-app-commands::
   :zephyr-app: samples/net/wifi/shell
   :board: ek_ra8m1
   :shield: renesas_qciot_rrq61051evz_pmod
   :goals: build flash

.. zephyr-app-commands::
   :zephyr-app: samples/net/wifi/shell
   :board: ek_ra6m4
   :shield: renesas_qciot_rrq61051evz_mikrobus
   :goals: build flash

References
**********

.. target-notes::

.. _Digilent Pmod:
   https://digilent.com/reference/pmod/start

.. _mikroBUS:
   https://www.mikroe.com/mikrobus
