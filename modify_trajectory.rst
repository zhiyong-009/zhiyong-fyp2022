=================
Modify trajectory
=================

.. image:: temp_static/modify_trajectory/modify_traj.png
   :align: center

Pour modifier une trajectoire on sélectionne d'abord les poses à modifier puis on choisit le type d'opération à réaliser sur ces poses.

Sélection des poses
===================

5 modes de sélection sont disponibles:

* **Whole trajectory:** Permet de sélectionner toute la trajectoire.
* **Trajectory:** Permet de sélectionner une ou plusieurs poses sur l’ensemble de la trajectoire.
* **Layers:** Permet de sélectionner une ou plusieurs couches de la trajectoire.
* **Within layer:** Permet de sélectionner une couche, d’y sélectionner une ou plusieurs poses et de propager la sélection à d’autres couches.
* **Filter by information:** Option permettant de selectionner toutes les poses ayant des paramètres ou informations équivalents (polygon_start, polygon_end...).

Within layer
------------

Uniquement dans le cas du mode ``Within layer``, une fenêtre de sélection apparaît. Cette fenêtre permet de choisir la couche dans laquelle on va sélectionner des poses (la 1ère couche possède le numéro 0).

.. image:: temp_static/modify_trajectory/pick_a_layer.png
   :align: center


La sélection des [poses / couches / poses à l'intérieur d'une couche] se fait de la même manière. Les poses sélectionnées sont entourés par des sphères grises légèrement transparentes.

* Le bouton ``All`` permet de sélectionner tous les éléments tandis que ``None`` efface la sélection.
* Le bouton ``Pair`` permet de sélectionner tous les éléments pairs tandis que ``Odd`` permet de sélectionner tous les éléments impairs.

Filter by information
---------------------
Cette option comprends les options suivantes:

* **Select all 'Start' poses** : permet de sélectionner toutes les poses ``polygon_start``
* **Select all 'End' poses** : permet de sélectionner toutes les poses ``polygon_end``
* **Select all 'Start' AND 'End' poses** : permet de sélectionner toutes les poses qui sont ``polygon_start`` **ET** ``polygon_end``
* **Select last pose of layers** : permet de sélectionner la dernière pose de toutes les couches

Il est possible de sélectionner toutes les poses ayant les mêmes propriétés; par exemple toutes les poses ``polygon_start``.

.. image:: temp_static/modify_trajectory/filter_poses_by_information.png
   :align: center

Ainsi, si l'option ``Select all 'Start' poses`` est sélectionnée, alors toutes les poses ``polygon_start`` seront sélectionnées dans la trajectoire :

.. image:: temp_static/modify_trajectory/filter_poses_by_information_eg.png
   :align: center

Sélection manuelle
------------------

.. image:: temp_static/modify_trajectory/check_box.png
   :align: center

Cochez individuellement les cases des éléments que vous souhaitez sélectionner.

Sélection de plages
-------------------

.. image:: temp_static/modify_trajectory/range.png
   :align: center

Ce mode permet de sélectionner rapidement une plage d'éléments. Indiquez l'index de l'élément de début et de fin de sélection. 3 options:

* ``Add``: Ajoute la plage d'éléments à la sélection (les éléments déjà sélectionnés le restent).
* ``Remove``: Enlève la plage d'éléments à la sélection (les éléments non sélectionnés le restent).
* ``Invert``: Inverse la sélection pour la plage d'éléments: les éléments sélectionnés ne le seront plus et inversement.

Type d'opération à réaliser
===========================

Une fois la sélection validée les sphères grises deviennent opaques et la fenêtre suivante apparaît:

.. image:: temp_static/modify_trajectory/operation.png
   :align: center

L'opération s'appliquera uniquement sur les poses sélectionnées précédemment.

Modify
------

Permet de modifier la pose, les paramètres remplis dans :ref:`Fill Trajectory` et les ``Polygon start``/``Polygon end`` des poses sélectionnées:

  .. |modify_1| image:: temp_static/modify_trajectory/modify_1.png
   :width: 50 %

  .. |modify_2| image:: temp_static/modify_trajectory/modify_2.png
   :width: 50 %

|modify_1| |modify_2|

Deux modes de modification existent:

* ``Relative``: Pour chaque pose sélectionnée la valeur entrée dans l'interface sera ajoutée au paramètre de la pose.
* ``Absolute``: Pour chaque pose sélectionnée la valeur entrée écrasera le paramètre de la pose.

Exemple avec ``Relative``, nous ajoutons `-100` à la valeur ``laser_power`` sur les pose `0` et `1` de la trajectoire. Trajectoire de base:


.. code-block:: YAML

    Pose 0
     - laser_power = 0
    Pose 1
     - laser_power = 1000
    Pose 2
     - laser_power = 200

Trajectoire obtenue:

.. code-block:: YAML

    Pose 0
     - laser_power = -100
    Pose 1
     - laser_power = 900
    Pose 2
     - laser_power = 200

Exemple avec ``Absolute``, nous modifions la valeur ``laser_power`` à `500` sur les poses `0` et `1` de la trajectoire. Trajectoire de base:

.. code-block:: YAML

    Pose 0
     - laser_power = 0
    Pose 1
     - laser_power = 1000
    Pose 2
     - laser_power = 200

Trajectoire obtenue:

.. code-block:: YAML

    Pose 0
     - laser_power = 500
    Pose 1
     - laser_power = 500
    Pose 2
     - laser_power = 200

Add
---
Ajoute une pose après chacune des poses sélectionnés.

Delete
------
Supprime les poses sélectionnés.

.. DANGER::
  Les poses supprimées ne peuvent pas être récupérées autrement qu'en utilisant le panneau :ref:`Trajectory utilities`.

.. ATTENTION::
  Si l’ensemble des poses formant une entrée ou une sortie est supprimé alors l'entrée et/ou la sortie seront de nouveaux générées à partir des derniers paramètres enregistrés dans le panneau :ref:`Entry and exit strategies`.

Reset
-----
Annule toutes les modifications sur les poses sélectionnées.

.. ATTENTION::
  Les poses qui ont été ajoutées avec :ref:`Add` seront supprimées.

Interruption
------------
Ce mode permet d'ajouter des interruptions du procédé; c'est à dire que la dépose de matière sera arrêtée puis reprise à cette endroit.
C'est utile pour ajouter des pauses dans le programme (refroidissement etc.).

Des trajectoires d'entrées et de sorties seront automatiquement ajoutés autour des points sélectionnés.

Paramètre **Add poses**:

* Si non coché la pose selectionnée sera convertie en **polygon_end** (arrêt du procédé) et la pose suivante dans la trajectoire sera convertie en **polygon_start** (démarrage du procédé).
* Si coché la pose sera dupliquée et la première sera modifiée en **polygon_end**, la deuxième en **polygon_start**.

Height between layers
---------------------
Ce mode permet de changer la hauteur entre les différentes couches de la trajectoire sélectionnée.

* Il suffit qu'une pose de la couche soit sélectionnée pour que la modification s'applique sur cette couche.
* La modification est toujours effectué entre la couche sélectionnée et la couche suivante. Il est donc inutile de sélectionner la dernière couche de la trajectoire; aucune modification ne sera effectuée.

Il suffit de sélectionner les couches que l'on souhaite modifier.

.. image:: temp_static/modify_trajectory/layer_height_selection.png
   :align: center

Deux modes de modification existent:

* ``Relative``: Pour chaque hauteur sélectionnée, la valeur entrée dans l'interface sera ajoutée à la hauteur existante de la couche.
* ``Absolute``: Pour chaque hauteur sélectionnée, la valeur entrée dans l'interface remplacera la hauteur actuellement en place.

.. image:: temp_static/modify_trajectory/modify_height_layer.png
   :align: center

Ici on a selectionné la couche 2 et le mode relatif avec -1 mm ce qui donne le résultat suivant:

.. image:: temp_static/modify_trajectory/layer_height_result.png
   :align: center

.. NOTE::
  La modification échouera si le résultat entraine une hauteur de couche négative ou nulle

Tweak blend radius
---------------------
Ce mode permet de modifier le ``Blend Radius`` de la trajectoire en fonction de l'angle courant.

Il suffit de sélectionner les couches ou les points que l'on souhaite modifier et d'y affecter la valeur souhaitée.

.. image:: temp_static/modify_trajectory/tweak_blend_radius.png
   :align: center

L'interface se compose de quatre colonnes:

* ``Alpha min``: Valeur de la plage angulaire minimale à prendre en compte pour la modification.
* ``Alpha max``: Valeur de la plage angulaire maximale exclue pour la modification (l'angle est exclu)
* ``Mode``

  * ``Relative``: Ajoute à la valeur courante le nouveau blend radius (valeurs négatives autorisées).
  * ``Absolute``: Remplace la valeur de blend radius.

* ``Blend radius``: Pour chaque sélection la nouvelle valeur du blend radius souhaitée.


.. NOTE::
  La valeur ``Alpha max`` est exclue de chaque ligne
  (e.g. ici le blend radius appliqué pour 100° sera 25, pas 20)

Deux boutons supplémentaires sont présents:

* ``Remove row``: Permet de retirer une ligne de valeurs.
* ``Add row``: Permet d'ajouter une ligne de valeurs.

.. NOTE::
  On peut afficher au minimum 1 ligne et jusqu'à 8 lignes.

Simplify
--------
Cette modification permet de simplifier la trajectoire en supprimant des points.

.. image:: temp_static/modify_trajectory/simplify_trajectory.png
 :align: center

.. image:: temp_static/modify_trajectory/simplify_trajectory_details.png
 :align: center

L'algorithme est basé sur le calcul de la surface d'un triangle formée par 3 points juxtaposés de la trajectoire.
Le sommet courant supprimé par cet algorithme représente la hauteur du triangle.

* ``Area threshold``: renseigne la surface minimale (exclue) du triangle en deçà de laquelle les sommets seront supprimés

.. image:: temp_static/modify_trajectory/simplify_trajectory_error.png
 :align: center

Push/pull angle
---------------
Cette modification change l'angle de la pose pour pencher l'outil en avant/arrière afin de pousser/tirer le fil.

.. ATTENTION::
  Cet algorithme ne fonctionne que pour des trajectoires planes (Z constant à l'intérieur d'une couche)

.. image:: temp_static/modify_trajectory/push_pull_before.png
 :align: center

.. image:: temp_static/modify_trajectory/push_pull_after.png
 :align: center

Rotate
------
Permet une rotation des poses sélectionnées:

.. image:: temp_static/modify_trajectory/rotate.png
 :align: center

La rotation n'est possible qu'autour de l'axe Z, les orientations des poses sont affectées par cette rotation.

Reflect
-------
Permet d’effectuer une symétrie autour d'un point:

.. image:: temp_static/modify_trajectory/reflect.png
 :align: center

Scale
-----
Permet de redimensionner (agrandir / réduire) des points suivant un centre.

.. image:: temp_static/modify_trajectory/scale.png
 :align: center

.. ATTENTION::
  Si des poses appartenant à plusieurs couches sont sélectionnées, l'écart entre les couches est préservé.

Shift
-----
Permet de décaler des poses suivant une direction et un angle.

.. image:: temp_static/modify_trajectory/shift.png
 :align: center

* ``Shift direction angle``: Dans quelle direction décaler chaque couche. 0° correspond à l'axe X, 90° l'axe Y, 180° la direction -X etc. Défini la direction du décalage dans le plan XY.
* ``Z shift angle``: L'angle de décalage de chaque couche selon l'axe Z. Plus cet angle est grand plus les couches sont décalées loin.
