=======
Display
=======

.. image:: temp_static/display/display.png
   :align: center

Ce panneau contient les paramètres d’affichage de la trajectoire.

.. NOTE::
  En aucun cas le paramétrage de ce panneau ne modifiera la trajectoire générée (exemple: le programme robot), ce sont uniquement des paramètres de visualisation.

Parameters
==========

* **Display type:** Affichage de la trajectoire avec des cylindres ou des fils.
* **Size:** La taile des cylindres/fils affichés
* **Color by:** Affecte une couleur par couche, vitesse etc. suivant le mode choisir

Axis
====

* **Enable:** Affiche des trièdres X/Y/Z (rouge, vert, bleu) correspondant à l'orientation de la pose
* **Size:** La taille du trièdre affiché

Labels
======

* **Enable:** Affiche du texte au dessus de chaque pose
* **Size:** La taille du texte affiché
* **Labels displayed:** Permet de choisir le contenu du texte affiché

.. image:: temp_static/display/labels_displayed.png
   :align: center

.. ATTENTION::
  Le mode d’affichage des poses est important lorsque l’on modifie une trajectoire:
   * En mode sélection ``Trajectory``: utiliser l’affichage ``Pose ID``
   * En mode sélection ``Layers``: utiliser l’affichage ``Layer level``
   * En mode sélection ``Within layer``: utiliser l’affichage ``Pose ID within layer``

Transparency
============

* **Enable:** Active le changement de transparence des poses

.. NOTE::
  Le mode d'affichage de la transparence est défini en pourcentage:
   * 100%: pose affichée
   * 50%: pose semi transparente
   * 0%: pose masquée

Boutons
=======

* **Pick mesh color:** Permet de choisir la couleur du maillage (si un maillage est affiché)
* **Display markers:** Actualise la trajectoire à l'écran
* **Clear markers:** Supprime la trajectoire et le maillage à l'écran
