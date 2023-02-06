================
Problèmes connus
================

L'application ne démarre pas
============================

Ouvrir un terminal et lancer:

.. code-block:: bash

   killall rviz roscore

Essayer de lancer l'application de nouveau. Si l'application ne démarre pas relancer la commande ``killall rviz roscore`` puis lancer:

.. code-block:: bash

   gtk-launch ros_additive_manufacturing

Copier la sortie du terminal et l'envoyer au support, voir :ref:`Contact`.

Impossible de sélectionner un fichier
=====================================

Si la fenêtre de sélection de fichier apparaît incomplète comme ci-dessous:

.. image:: temp_static/known_issues/open_file.png
   :scale: 100 %
   :align: center

Pour corriger le problème:

* Fermer la fenêtre d'ouverture de fichier.
* Dans le panneau :ref:`Display` cliquer sur le bouton ``Clear``.
* Ouvrir le fichier.
* Il est possible d'afficher de nouveau les marqueurs visuels.
