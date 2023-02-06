======
Frames
======

.. image:: temp_static/frames/frames.png
   :align: center

.. NOTE::
  Configurez ``Display mode`` du panneau :ref:`Display` pour afficher les axes lorsque l'on modifie le ``Tool orientation``

.. ATTENTION::
  Lors d'une modification du repère ``Tool orientation`` il est nécessaire de mettre à jour la visualisation manuellement pour que la modification soit visible.

Arborescence du système de coordonnées
======================================
Si vous déplacez un repère, vous déplacerez aussi tous ceux qui héritent de lui.

.. code-block:: YAML

  - base # Repère fixe "monde"
    - trajectory_frame # Repère trajectoire
      - start_pose # Pose de départ
        - tool_orientation # Orientation de l'outil sur la trajectoire

.. image:: temp_static/frames/coordinates_system.png
   :scale: 50 %
   :align: center

.. NOTE::
  Il est possible d'inspecter l'arbre des repères dans le panneau :ref:`Displays` (objet ``TF``)

* On ne peut pas bouger le repère ``base``.
* Le repère ``trajectory_frame`` est généralement construit sur un marbre ou directement sur une pièce mécanique.
* Le repère ``start_pose`` défini un point de départ de la trajectoire.
* Le repère ``tool_orientation`` sert à définir l'orientation de l'effecteur sur la trajectoire.
