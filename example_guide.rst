=====================
Exemple d’utilisation
=====================

Pour commencer, dans le panneau :ref:`Path planning`, générez une trajectoire avec l’algorithme ``DonghongDing`` à partir du fichier maillage ``ros_additive_manufacturing/ram_path_planning/meshes/inversed_pyramid.ply``:

Pour choisir le ficher cliquez sur les trois petits points sous ``YAML or mesh file:``:

.. image:: temp_static/example_guide/choose_YAML_or_mesh_file.png
   :align: center

Dans le dossier ``meshes``, et ouvrez ``inversed_pyramid.ply``. Cliquez sur ``Generate Trajectory`` la trajectoire suivante devrait être générée:

.. image:: temp_static/example_guide/generate_1.png
   :scale: 30 %
   :align: center

Grâce aux informations fournies dans :ref:`Fenêtre principale`, tournez la caméra pour observer la trajectoire:

.. image:: temp_static/example_guide/generate_2.png
   :scale: 30 %
   :align: center

Observer les valeurs des champs du panneau :ref:`Trajectory information`. Modifiez les informations dans :ref:`Path planning` comme sur l'image ci-dessous et régénérez la trajectoire:

.. image:: temp_static/example_guide/generate_3.png
   :scale: 30 %
   :align: center

Les modifications apportées ont entraîné un nouveau calcul des informations dans le panneau :ref:`Trajectory information`.

Dans le panneau :ref:`Entry and exit strategies`, remplissez les paramètres comme sur la capture et régénérez la trajectoire:

.. image:: temp_static/example_guide/entry_example.png
   :scale: 30 %
   :align: center

La trajectoire marron en haut à gauche de l’affichage correspond à la trajectoire de sortie matière tandis que la barre rouge en bas à gauche correspond à l'entrée matière. Attention! Ne pas confondre avec les trièdres rouges / verts / bleus que l’on voit dépasser à la base de la pièce, ils correspondent aux repères pour positionner la pièce.

Modifiez des valeurs dans le panneau :ref:`Fill trajectory` et générez des trajectoires. Observer les valeurs des champs des poses grâce au panneau :ref:`Pose information`.
Il est normal que sur les poses où ``Entry pose`` / ``Exit pose`` est à la valeur ``True`` que ``Laser power`` soit à 0 Watts et que ``Feed rate`` soit à 0 mètres/min, on ne commence à déposer de la matière qu'après une pose de type ``Entry pose`` ces valeurs ne sont effectives que lorsque l'on dépose de la matière.

Dans le panneau :ref:`Display` testez les différentes options afin de voir les changements dans l’affichage. Utilisez ``Display mode`` avec ``Cylinders + axis mode`` et l’option ``Display labels`` pour la suite de l’exemple. Vous devriez voir ceci:

.. image:: temp_static/example_guide/generate_4.png
   :scale: 30 %
   :align: center

Dans le panneau :ref:`Frames`, lorsque vous modifiez ``Robot trajectory frame`` ou ``Start pose`` vous pouvez voir les déplacements des axes en direct. Lorsque vous modifiez ``Tool orientation`` il est nécessaire de générer de nouveau la trajectoire pour que les modifications soient effectives. Vous devriez voir que les trièdres de toutes les poses ont changé.

.. image:: temp_static/example_guide/generate_5.png
   :scale: 30 %
   :align: center

Dans le panneau :ref:`Display`, affichez la trajectoire en mode ``Cylinders mode``. Dans le panneau :ref:`Modify trajectory` sélectionnez ``Layers``, puis cliquez sur ``Start selection``. Choisissez les couches 2 et 3, vous pouvez afficher uniquement les couches que vous souhaitez modifier avec le panneau :ref:`Display`. Vous pouvez voir que des sphères grises transparentes apparaissent autour de vos sélections:

.. image:: temp_static/example_guide/generate_6.png
   :scale: 30 %
   :align: center

Lorsque vous validez votre sélection, les sphères deviennent opaques.

.. image:: temp_static/example_guide/generate_7.png
   :scale: 30 %
   :align: center

Choisissez l’option ``Modify`` et validez. Pour pouvoir observer les modifications que vous allez effectuer affichez la pose 71 dans le panneau :ref:`Pose information`. Modifiez les paramètres comme sur la capture ci-dessous:

.. image:: temp_static/example_guide/generate_8.png
   :scale: 70 %
   :align: center

Validez la modification. Ta trajectoire a été modifiée, vous pouvez maintenant voir dans :ref:`Pose information` les modifications effectuées.
Avant que vous puissiez tester les différentes modifications possibles, dans les panneaux de droite, allez dans :ref:`Trajectory utilities` et appuyez sur ``Back``. Dans :ref:`Pose information` retournez sur la pose 71, vous verrez que les modifications effectuées ne sont plus présentes. Vous pouvez revenir sur les modifications en appuyant sur ``Forward``. Vous pouvez vous baladez entre les modifications de la trajectoire avec ces boutons.
Maintenant vous pouvez tester les différentes modifications possibles du panneau :ref:`Modify trajectory`.

.. ATTENTION::
     Si vous revenez en arrière avec ``Back`` et que vous effectuez d’autres modifications, les modifications précédentes qui étaient dans ``Forward`` seront écrasées.
