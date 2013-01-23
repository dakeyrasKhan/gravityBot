gravityBot
==========

Path .h/c-pp
------------

On créé un path en lui donnant un vecteur de Vector. Ceux -ci doivent supporter :
- méthode : normalize -> rend le vecteur normé
- addition soustraction multiplication par un scalaire
- méthode : norm -> attention à la 2pi-périodicité

Méthodes de path :
- Path(vector<Position>& waypoints) initialise le path et affecte les waypoints.
- getNextPosition(Vector speed, double deltaT) renvoit la prochaine position. speed contient les vitesses linéaires et angulaires.
- isDone renvoit true si on a finit le chemin.

**Si on a finit le chemin, getNextPosition renvoit la position actuelle**


Le Vector*Vector, c'est la multiplication composante par composante ?
-> oui

