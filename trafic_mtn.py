import pygame as py
import pygame.gfxdraw as gfx
import numpy as np
from scipy.spatial import distance  # inutile rn
from math import atan, pi, sin, cos, radians, inf, floor, sqrt
import math
import matplotlib.pyplot as plt
import random as rd
import networkx as nx
from shapely.geometry import Polygon
import sys
import time






blanc=(255,255,255)
noir=(0,0,0)
gris=(170,170,170)
gris_clair=(220,220,220)
rouge=(255,0,0)
vert=(0,255,0)
cyan=(43,240,240)
bleu_gris=(142, 162, 198)


liste_cardinaux=["nord","sud","ouest","est"]

epaisseur=30


largeur_carte=3000
hauteur_carte=3000

facteur=0.7
largeur_affichage=floor(1920*facteur)
hauteur_affichage=floor(1080*facteur)

marge=-10

tracer=False


def clamp(x,minimum,maximum):
    if x<minimum:
        x=minimum
    elif x>maximum:
        x=maximum
    return(x)

#def __init__(self,config={}):   # config: dictionnaire

class Camera:
    def __init__(self,pos:()):
        self.pos=pos
        self.x,self.y=pos
        self.vitesse=15




class Simulation:
    def __init__(self,fps):
        self.initialiser_parametres()
        self.fps=fps
        dessiner=True
        self.penalite_score_collision=0.5

    def initialiser_parametres(self):
        self.temps=0.0
        self.nb_images=0
        self.dt=1/60

        self.stop=False





        self.dico_points={}     # stockage des points {nom:Point}

        self.liste_routes=[]    # stockage des routes Route
        self.liste_voitures=[]  # stockage des voitures Voiture
        self.liste_generateurs=[]  # stockage des générateurs de voitures Generateur
        self.liste_feux_circulation=[]  # stockages des feux de circulation Feu_circulation
        self.liste_joints=[]     # stockage des joints Point (qui est début d'une route)
        self.liste_compteurs=[]     # stockage des compteurs Compteurs
        self.liste_arrivees=[]  # stockage des arrivees Point
        self.liste_intersections=[]     # stockage des intersections Intersection

        self.liste_vitesses=[]
        self.liste_accelerations=[]
        self.liste_vitesses_moyennes_normalisees=[]
        self.liste_vitesses_max=[]


        # à retravailler
        self.liste_temps_feu_circulation=[]
        self.liste_nb_voitures_passees=[]

        self.liste_densites=[]
        self.liste_debits=[]
        self.liste_temps=[]



        self.nb_voitures_arrivees=0



    def initialiser_liste_routes(self,ROUTES):
        for noms in ROUTES:
            self.initialiser_route(self.dico_points[noms[0]],self.dico_points[noms[1]])

        self.longueur_totale_routes=0
        for route in self.liste_routes:
            self.longueur_totale_routes+=route.longueur

    def initialiser_liste_generateurs(self,GENERATEURS):
        for generateur in GENERATEURS:
            self.initialiser_generateur(generateur[0],generateur[1],generateur[2])

    def initialiser_dico_points(self,DICO_POINTS):
        for nom in DICO_POINTS:
            self.intialiser_point(nom,DICO_POINTS[nom])

    def initialiser_liste_feux_circulation(self,FEUX_CIRCULATION):
        for feu_circulation in FEUX_CIRCULATION:

            route=self.route_associee(*feu_circulation[0])
            placement_route,frequence,etat=feu_circulation[1],feu_circulation[2],feu_circulation[3]

            self.initialiser_feu_circulation(route,placement_route,frequence,etat)  # feu_circulation=((nom,nom),placement_route,frequence)

    def initialiser_liste_compteurs(self,LISTE_COMPTEURS):
        for compteur in LISTE_COMPTEURS:
            self.initialiser_compteur(*compteur)


    def initialiser_route(self,point_debut,point_fin):
        route=Route(point_debut,point_fin)
        route.type="simple"
        self.liste_routes.append(route)
        return(route)

    def initialiser_generateur(self,nom:(),noms_points:(),frequence):
        generateur=Generateur(self.pos_depuis_nom_point(nom),self.route_associee(*noms_points),frequence)
        self.liste_generateurs.append(generateur)

    def intialiser_point(self,nom,pos):
        point=Point(pos)
        self.dico_points[nom]=point

    def initialiser_feu_circulation(self,route,placement_route,frequence,etat):
        feu_circulation=Feu_circulation(route,placement_route,frequence,etat)
        self.liste_feux_circulation.append(feu_circulation)
        route.liste_feux_circulation.append(feu_circulation)

    def supprimer_elements_simulation(self):
        for route in self.liste_routes:
            del route
        for feu_circulation in self.liste_feux_circulation:
            del feu_circulation
        self.dico_points={}
        for voiture in self.liste_voitures:
            del voiture
        for joint in self.liste_joints:
            del joint
        for compteur in self.liste_compteurs:
            del compteur
        for arrivee in self.liste_arrivees:
            del arrivee
        for intersection in self.liste_intersections:
            for cardinal in liste_cardinaux:
                for point in intersection.dico_points[cardinal]:
                    del point
            del intersection

        self.initialiser_parametres()


    def initialiser_compteur(self,nom_route,placement_route):
        route=self.route_associee(*nom_route)
        compteur=Compteur(route,placement_route)
        self.liste_compteurs.append(compteur)

    def initialiser_voiture(self,x,y, route):
        voiture = Voiture(x,y,route)
        self.liste_voitures.append(voiture)
        return(voiture)

    def initialiser_matrice_adjacence(self):
        self.taille_matrice_adjacence=len(self.dico_points)
        self.matrice_adjacence=np.zeros((self.taille_matrice_adjacence, self.taille_matrice_adjacence),dtype=np.uint8)
        for i,point_debut in self.dico_points.items():
                for j,point_fin in self.dico_points.items():
                    for route in self.liste_routes:
                        if route.point_debut==point_debut and route.point_fin == point_fin:
                            self.matrice_adjacence[i][j]=1

    def initialiser_joints(self):
        # if afficher_points_inutiles==False:   # à changer
        if 1:
            for route in self.liste_routes:
                self.liste_joints.append(route.point_debut)
        else:
            for nom in self.dico_points:
                self.liste_joints.append(self.dico_points[nom])

    def initialiser_arrivees(self):
        points_debuts=[]
        for route in self.liste_routes:
            points_debuts.append(route.point_debut)
        for route in self.liste_routes:
            if not route.point_fin in points_debuts:
                self.liste_arrivees.append(Arrivee(route.point_fin))

    def initialiser_intersections(self):    # detecter s'il y a une intersection

        # obtention d'un dictionnaire {route_entrante:liste_routes_sortantes} avec route_entrante unique
        dico_routes={}
        for route_entree in self.liste_routes:
            compte=0
            liste_routes_sorties=[]
            for route_sortie in self.liste_routes:
                if route_entree.point_fin==route_sortie.point_debut:
                    compte+=1
                    liste_routes_sorties.append(route_sortie)
            if len(liste_routes_sorties)>=1:    # seuil de détection d'une intersection
                dico_routes[route_entree]=liste_routes_sorties

        # regroupement des routes entrantes
        groupes = {}
        for cle, valeurs in dico_routes.items():
            if tuple(valeurs) not in groupes:
                groupes[tuple(valeurs)]=[cle]
            else:
                groupes[tuple(valeurs)].append(cle)

        liste_routes=[(valeur_commune,[*cles]) for cles, valeur_commune in groupes.items()]    # [(routes_entrees,routes_sorties),...]

        #print(liste_routes)
        for couple in liste_routes:
            self.initialiser_intersection(couple[0][0].point_fin.pos,*couple)




        #self.initialiser_listes_points_intersection(self.liste_intersections[0],*liste_routes[0])
        #self.liste_intersections[0].initialiser_dico_points()

    def initialiser_intersection(self,pos,liste_routes_entrantes,liste_route_sortantes):
        intersection=Intersection(pos)
        self.initialiser_listes_points_intersection(intersection,liste_routes_entrantes,liste_route_sortantes)  # comprendre quel route rentre/sort de quel coté (cardinal)
        intersection.initialiser_dico_points()  # placer points autour de intersection
        intersection.tri_dico_routes()  # trier les points dans le bon ordre
        intersection.initialiser_proprietes()
        intersection.tourner_points()
        intersection.separer_routes()
        self.liste_intersections.append(intersection)


    def initialiser_listes_points_intersection(self,intersection,liste_routes_entrantes,liste_routes_sortantes):

        # met à jour dico_routes_entrantes/sortantes["cardinal"]
        for route_entrante in liste_routes_entrantes:
            delta_x=route_entrante.point_debut.x-intersection.x
            delta_y=route_entrante.point_debut.y-intersection.y

            if delta_x>=0 and delta_y>=0:
                if delta_x>=delta_y:
                    intersection.nb_points_est+=1
                    route_entrante.arrivee_intersection="est"
                    intersection.dico_routes_entrantes["est"].append(route_entrante)
                else:
                    intersection.nb_points_sud+=1
                    route_entrante.arrivee_intersection="sud"
                    intersection.dico_routes_entrantes["sud"].append(route_entrante)

            elif delta_x>=0 and delta_y<=0:
                if delta_x>=-delta_y:
                    intersection.nb_points_est+=1
                    route_entrante.arrivee_intersection="est"
                    intersection.dico_routes_entrantes["est"].append(route_entrante)
                else:
                    intersection.nb_points_nord+=1
                    route_entrante.arrivee_intersection="nord"
                    intersection.dico_routes_entrantes["nord"].append(route_entrante)

            elif delta_x<=0 and delta_y>=0:
                if delta_x<=-delta_y:
                    intersection.nb_points_ouest+=1
                    route_entrante.arrivee_intersection="ouest"
                    intersection.dico_routes_entrantes["ouest"].append(route_entrante)
                else:
                    intersection.nb_points_sud+=1
                    route_entrante.arrivee_intersection="sud"
                    intersection.dico_routes_entrantes["sud"].append(route_entrante)

            elif delta_x<=0 and delta_y<=0:
                if delta_x<=delta_y:
                    intersection.nb_points_ouest+=1
                    route_entrante.arrivee_intersection="ouest"
                    intersection.dico_routes_entrantes["ouest"].append(route_entrante)
                else:
                    intersection.nb_points_nord+=1
                    route_entrante.arrivee_intersection="nord"
                    intersection.dico_routes_entrantes["nord"].append(route_entrante)


        for route_sortante in liste_routes_sortantes:
            delta_x=route_sortante.point_fin.x-intersection.x
            delta_y=route_sortante.point_fin.y-intersection.y

            if delta_x>=0 and delta_y>=0:
                if delta_x>=delta_y:
                    intersection.nb_points_est+=1
                    route_sortante.arrivee_intersection="est"
                    intersection.dico_routes_sortantes["est"].append(route_sortante)
                else:
                    intersection.nb_points_sud+=1
                    route_sortante.arrivee_intersection="sud"
                    intersection.dico_routes_sortantes["sud"].append(route_sortante)

            elif delta_x>=0 and delta_y<=0:
                if delta_x>=-delta_y:
                    intersection.nb_points_est+=1
                    route_sortante.arrivee_intersection="est"
                    intersection.dico_routes_sortantes["est"].append(route_sortante)
                else:
                    intersection.nb_points_nord+=1
                    route_sortante.arrivee_intersection="nord"
                    intersection.dico_routes_sortantes["nord"].append(route_sortante)

            elif delta_x<=0 and delta_y>=0:
                if delta_x<=-delta_y:
                    intersection.nb_points_ouest+=1
                    route_sortante.arrivee_intersection="ouest"
                    intersection.dico_routes_sortantes["ouest"].append(route_sortante)
                else:
                    intersection.nb_points_sud+=1
                    route_sortante.arrivee_intersection="sud"
                    intersection.dico_routes_sortantes["sud"].append(route_sortante)

            elif delta_x<=0 and delta_y<=0:
                if delta_x<=delta_y:
                    intersection.nb_points_ouest+=1
                    route_sortante.arrivee_intersection="ouest"
                    intersection.dico_routes_sortantes["ouest"].append(route_sortante)
                else:
                    intersection.nb_points_nord+=1
                    route_sortante.arrivee_intersection="nord"
                    intersection.dico_routes_sortantes["nord"].append(route_sortante)



        #print(intersection.nb_points_nord,intersection.nb_points_sud,intersection.nb_points_ouest,intersection.nb_points_est)
        #intersection.initialiser_proprietes()

    def actualiser_routes_aux_intersections(self):  # bézier, routes dans les intersections (y compris aux bords)

        for intersection in self.liste_intersections:
            points_debut=[]
            points_fin=[]
            for cardinal in liste_cardinaux:

                for route in intersection.dico_routes[cardinal]:
                    indice=intersection.dico_routes[cardinal].index(route)
                    if intersection.est_entrante(route)==True:
                        route.point_fin=intersection.dico_points[cardinal][indice]
                        points_debut.append(route.point_fin)
                    else:
                        route.point_debut=intersection.dico_points[cardinal][indice]
                        points_fin.append(route.point_debut)

            for i in points_debut:
                for j in points_fin:
                    liste_points=self.courbe_bezier(j,i,Point((intersection.x,intersection.y)))
                    for k in range(0,len(liste_points)-1):
                        route_1=self.initialiser_route(i,liste_points[0])
                        route_2=self.initialiser_route(liste_points[k],liste_points[k+1])
                        route_3=self.initialiser_route(liste_points[-1],j)
                        route_1.type="sous_route"
                        route_2.type="sous_route"
                        route_3.type="sous_route"

    def point_controle(self,point1,point2,centre_x,centre_y):   # pour courbe Bézier
        x=0.5*(point1.x+point2.x)+centre_x
        y=0.5*(point1.y+point2.y)+centre_x
        return(Point((x,y)))

    def actualiser_routes(self):
        for route in self.liste_routes:
            route.initialiser_proprietes()
            self.initialiser_liste_routes_proches(route)
            route.liste_routes_adjacentes=self.routes_adjacentes(route)


    def initialiser_liste_routes_proches(self,route):
        portee=50
        portee2=portee**2
        for point in route.liste_points:
            for route_bis in self.liste_routes:

                ok=False
                for point_bis in route_bis.liste_points:
                    if ok==True:
                        break
                    else:
                        if abs(point.x-point_bis.x)**2+abs(point.y-point_bis.y)**2<portee2:
                            if not route_bis in route.liste_routes_proches:
                                route.liste_routes_proches.append(route_bis)
                                ok=True



    def actualiser_feux_circulation(self):
        for feu_circulation in self.liste_feux_circulation:
            feu_circulation.initialiser_proprietes()

    def actualiser_compteurs(self):
        for compteur in self.liste_compteurs:
            compteur.initialiser_proprietes()



    def initialiser_reste(self):

        self.initialiser_arrivees()
        self.initialiser_intersections()
        self.actualiser_routes_aux_intersections()


        self.actualiser_routes()
        self.initialiser_joints()
        self.actualiser_feux_circulation()
        self.actualiser_compteurs()

    def routes_adjacentes(self,route):  # O(longueur_liste_route)
        routes_adjacentes=[]
        for route_potentielle in self.liste_routes:
            if route_potentielle!=route and route_potentielle.point_debut==route.point_fin:
                routes_adjacentes.append(route_potentielle)
        return(routes_adjacentes)

    def barycentre(self,point1,point2,t):
        x=point1.x*t+point2.x*(1-t)
        y=point1.y*t+point2.y*(1-t)
        return(Point((x,y)))

    def courbe_bezier(self,point_debut,point_fin,point_controle):
        liste_t=np.linspace(0,1,6)
        liste_points=[]

        for t in liste_t:
            if t!=0 and t!=1:
                T=self.barycentre(self.barycentre(point_debut,point_fin,t),self.barycentre(point_controle,point_fin,t),t)
                liste_points.append(T)
        return(liste_points)

    def route_associee(self,nom1,nom2):
        point1=self.dico_points[nom1]
        point2=self.dico_points[nom2]
        for route in self.liste_routes:
            if route.point_debut==point1 and route.point_fin==point2:
                return(route)
        print("route non existante")

    def pos_depuis_nom_point(self,nom):
        return(self.dico_points[nom].pos)
        print("nom non existant")


    def nom_point(self,point):
        for nom in self.dico_points:
            if point==self.dico_points[nom]:
                return(nom)
        print("nom_point(): nom non existant")

class Generateur:
    def __init__(self,pos:(),route,frequence):
        self.pos=pos
        self.x, self.y=self.pos
        self.frequence=frequence
        self.attente=0
        self.route=route
        self.taille=(50,65)
        self.angle=self.route.angle

        self.polygone=None

        self.derniere_voiture=None  # qu'il a sorti

        self.initialiser_polygone()

    def initialiser_polygone(self):
        longueur=self.taille[0]
        largeur=self.taille[1]

        # polygone
        liste_sommets=[(self.x-longueur/2,self.y-largeur/2),(self.x+longueur/2,self.y-largeur/2),(self.x+longueur/2,self.y+largeur/2),(self.x-longueur/2,self.y+largeur/2)]
        sommets_orientes=self.rotate_rectangle(liste_sommets,(self.x,self.y),self.angle)

        self.polygone=Polygon(sommets_orientes)

    def rotate_point(self,point, center, angle):
        """Rotation d'un point autour d'un centre donné avec un angle en radians."""
        rotated_x = math.cos(angle) * (point[0] - center[0]) - math.sin(angle) * (point[1] - center[1]) + center[0]
        rotated_y = math.sin(angle) * (point[0] - center[0]) + math.cos(angle) * (point[1] - center[1]) + center[1]
        return (rotated_x, rotated_y)

    def rotate_rectangle(self,rectangle, center, angle):
        """Rotation des sommets d'un rectangle autour d'un centre donné avec un angle en radians."""
        rotated_vertices = []
        for vertex in rectangle:
            rotated_vertex = self.rotate_point(vertex, center, angle)
            rotated_vertices.append(rotated_vertex)
        return rotated_vertices



    def doit_creer(self):
        if self.attente>=self.frequence and self.verifier_peut_creer_voiture():
            return(True)
        else:
            return(False)

    def mettre_a_jour_derniere_voiture(self):
        if self.derniere_voiture!=None:
            # if not self.polygone.intersects(self.derniere_voiture.polygone):
            #     self.derniere_voiture=None
            if self.derniere_voiture.avancement>=40:
                self.derniere_voiture=None

    def verifier_peut_creer_voiture(self):
        if self.derniere_voiture==None:
            return(True)
        else:
            return(False)


class Feu_circulation:
    def __init__(self,route,placement_route,frequence,etat):
        self.route=route
        self.placement_route=placement_route
        self.frequence=frequence
        self.liste_etats=["rouge","vert"]
        self.etat=etat
        self.attente=0

        self.initialiser_proprietes()

    def initialiser_proprietes(self):
        self.x=self.route.point_debut.x+self.placement_route*self.route.longueur*cos(self.route.angle)
        self.y=self.route.point_debut.y+self.placement_route*self.route.longueur*sin(self.route.angle)
        self.pos=(self.x,self.y)

    def verifier_changement_etat(self):
        if self.attente>=self.frequence:
            if self.etat=="rouge":
                self.etat="vert"
            else:
                self.etat="rouge"
            self.attente=0

    def changer_etat(self,etat):
        self.etat=self.liste_etats[self.liste_etats.index(etat)]

    def augmenter_frequence(self):
        self.frequence+=1

    #def mettre_a_jour(self,dt):


class Point:
    def __init__(self,pos):
        self.pos=pos
        self.x, self.y=pos

class Intersection:
    def __init__(self,pos):
        self.pos=pos
        self.x, self.y=pos
        self.type="carrefour"
        self.nb_points_nord=0
        self.nb_points_sud=0
        self.nb_points_ouest=0
        self.nb_points_est=0
        self.dico_points={}     # points placés autour de l'intersection ; {"nord":[Point,Point],...} ; positions des points triés par ordre croissant selon l'axe de la composante qui varie

        self.dico_routes_entrantes={"nord":[],"sud":[],"ouest":[],"est":[]}
        self.dico_routes_sortantes={"nord":[],"sud":[],"ouest":[],"est":[]}







    def initialiser_proprietes(self):
        self.nb_points=self.nb_points_nord+self.nb_points_sud+self.nb_points_ouest+self.nb_points_est

        x_total=0
        points=[valeur for cle,valeur in self.dico_points.items()]
        for i in points:
            for j in i:
                x_total+=j.x
        self.x_pondere=x_total/self.nb_points

        y_total=0
        points=[valeur for cle,valeur in self.dico_points.items()]
        for i in points:
            for j in i:
                y_total+=j.y
        self.y_pondere=y_total/self.nb_points


    def initialiser_dico_points(self):
        self.taille=max([self.nb_points_nord,self.nb_points_sud,self.nb_points_ouest,self.nb_points_est])*epaisseur+70
        coef_cote=2



        # nord
        if self.nb_points_nord==1:
            self.dico_points["nord"]=[Point((self.x,self.y-self.taille//2))]
        else:
            self.dico_points["nord"]=[Point((x,self.y-self.taille//2)) for x in np.linspace(self.x-self.taille//coef_cote+epaisseur,self.x+self.taille//coef_cote-epaisseur,self.nb_points_nord)]

        # sud
        if self.nb_points_sud==1:
            self.dico_points["sud"]=[Point((self.x,self.y+self.taille//2))]
        else:
            self.dico_points["sud"]=[Point((x,self.y+self.taille//2)) for x in np.linspace(self.x-self.taille//coef_cote+epaisseur,self.x+self.taille//coef_cote-epaisseur,self.nb_points_sud)]

        # ouest
        if self.nb_points_ouest==1:
            self.dico_points["ouest"]=[Point((self.x-self.taille//2,self.y))]
        else:
            self.dico_points["ouest"]=[Point((self.x-self.taille//coef_cote,y)) for y in np.linspace(self.y-self.taille//coef_cote+epaisseur,self.y+self.taille//coef_cote-epaisseur,self.nb_points_ouest)]

        # est
        if self.nb_points_est==1:
            self.dico_points["est"]=[Point((self.x+self.taille//2,self.y))]
        else:
            self.dico_points["est"]=[Point((self.x+self.taille//coef_cote,y)) for y in np.linspace(self.y-self.taille//coef_cote+epaisseur,self.y+self.taille//coef_cote-epaisseur,self.nb_points_est)]

        # print(self.dico_points)




    def tourner_points(self):   # après avoir calculé le centre pondéré
        # orientation des points
        angle=0
        for cardinal in liste_cardinaux:
            for point in self.dico_points[cardinal]:
                point.x,point.y=self.tourner_point((point.x,point.y),(self.x_pondere,self.y_pondere),angle)
                point.pos=(point.x,point.y)


    def tourner_point(self,pos,centre,angle):
        """Rotation d'un point autour d'un centre donné avec un angle en radians."""
        x_oriente=cos(angle)*(pos[0]-centre[0])-sin(angle)*(pos[1]-centre[1])+centre[0]
        y_oriente=sin(angle)*(pos[0]-centre[0])+cos(angle)*(pos[1]-centre[1])+centre[1]
        return (x_oriente,y_oriente)


    def point_x(self,route):
        if self.est_entrante(route)==True:
            return(route.point_debut.x)
        else:
            return(route.point_fin.x)
    def point_y(self,route):
        if self.est_entrante(route)==True:
            return(route.point_debut.y)
        else:
            return(route.point_fin.y)


    def tri_dico_routes(self):
        self.dico_routes={}
        for cardinal in liste_cardinaux:
            self.dico_routes[cardinal]=self.dico_routes_entrantes[cardinal]+self.dico_routes_sortantes[cardinal]

            if cardinal=="nord" or cardinal=="sud":
                self.dico_routes[cardinal].sort(key=self.point_x)
            elif cardinal=="ouest" or cardinal=="est":
                self.dico_routes[cardinal].sort(key=self.point_y)
        #print(self.dico_routes)

    def separer_routes(self):   # entrante/sortante
        for cardinal in liste_cardinaux:
            for route in self.dico_routes[cardinal]:
                if self.est_entrante(route)==True:
                    self.dico_routes_entrantes[cardinal].append(route)
                else:
                    self.dico_routes_sortantes[cardinal].append(route)


    def est_entrante(self,route):
        if (route.point_fin.x,route.point_fin.y)==(self.x,self.y):
            return(True)
        else:
            return(False)

    # def tri_dico_routes_entrantes(self):
    #     for cardinal in liste_cardinaux:
    #         if cardinal=="nord" or cardinal=="sud":
    #             self.dico_routes_entrantes[cardinal].sort(key=self.point_x_debut)
    #         elif cardinal=="ouest" or cardinal=="est":
    #             self.dico_routes_entrantes[cardinal].sort(key=self.point_y_debut)
    #
    # def tri_dico_routes_sortantes(self):
    #     for cardinal in liste_cardinaux:
    #         if cardinal=="nord" or cardinal=="sud":
    #             self.dico_routes_sortantes[cardinal].sort(key=self.point_x_fin)
    #         elif cardinal=="ouest" or cardinal=="est":
    #             self.dico_routes_sortantes[cardinal].sort(key=self.point_y_fin)

class Arrivee:
    def __init__(self,point):
        self.x=point.x
        self.y=point.y
        self.pos=(self.x,self.y)
        self.taille=(60,60)


class Compteur:
    def __init__(self,route,placement_route):
        self.route=route
        self.placement_route=placement_route    # dans [0,1]
        self.x=self.route.point_debut.x+self.route.longueur*self.placement_route*cos(self.route.angle)
        self.y=self.route.point_debut.y+self.route.longueur*self.placement_route*sin(self.route.angle)


        self.taille=(10,40)

        self.compteur=0
        self.marge=10   # pas trop petit, dans [0,1]



        self.en_train_de_compter=None
        self.reinitialiser_compteur()

    def initialiser_proprietes(self):
        self.x=self.route.point_debut.x+self.route.longueur*self.placement_route*cos(self.route.angle)
        self.y=self.route.point_debut.y+self.route.longueur*self.placement_route*sin(self.route.angle)

    def reinitialiser_compteur(self):
        self.compteur=0

    def mettre_a_jour(self):
        if self.en_train_de_compter==None:
            for voiture in self.route.liste_voitures:
                if self.verifier_capter(voiture)==True:
                    self.en_train_de_compter=voiture
                    break
        if self.en_train_de_compter!=None and self.verifier_capter(self.en_train_de_compter)==False:
            self.compteur+=1
            self.en_train_de_compter=None

    def verifier_capter(self,voiture):
        if voiture.avancement+self.marge>=self.placement_route*self.route.longueur and voiture.avancement-self.marge<=self.placement_route*self.route.longueur:
            return(True)
        else:
            return(False)


class Route:
    def __init__(self,point_debut,point_fin):
        self.point_debut=point_debut
        self.point_fin=point_fin

        self.liste_voitures=[]
        self.liste_feux_circulation=[]

        self.nb_points=5
        self.liste_points=[]    # points intermédiaires qui composent la route

        self.liste_routes_proches=[]
        self.liste_routes_adjacentes=[]

        self.initialiser_proprietes()

        self.type=None  # pas complet

        self.liste_sommets=[]

    def initialiser_proprietes(self):

        # longueur et angle
        self.longueur=distance.euclidean(self.point_debut.pos,self.point_fin.pos)
        if self.point_fin.x>self.point_debut.x:
            self.angle=atan((self.point_fin.y-self.point_debut.y)/(self.point_fin.x-self.point_debut.x))
        elif self.point_fin.x<self.point_debut.x:
            self.angle=atan((self.point_fin.y-self.point_debut.y)/(self.point_fin.x-self.point_debut.x))+pi
        else:
            if self.point_fin.y>self.point_debut.y:
                self.angle=pi/2
            else:
                self.angle=-pi/2

        # liste des points de la route
        liste_x=np.linspace(self.point_debut.x,self.point_fin.x,self.nb_points)
        liste_y=np.linspace(self.point_debut.y,self.point_fin.y,self.nb_points)
        for i in range(len(liste_x)):
            self.liste_points.append(Point((liste_x[i],liste_y[i])))

        self.initialiser_liste_sommets()


    def initialiser_liste_sommets(self):
        centre=(self.point_debut.x+self.point_fin.x)/2,(self.point_debut.y+self.point_fin.y)/2
        longueur=distance.euclidean(self.point_debut.pos,self.point_fin.pos)  # Total longueur of line
        angle = math.atan2(self.point_debut.y - self.point_fin.y, self.point_debut.x-self.point_fin.x)

        UL = (centre[0] + (longueur/2.) * cos(angle) - (epaisseur/2.) * sin(angle),
            centre[1] + (epaisseur/2.) * cos(angle) + (longueur/2.) * sin(angle))
        UR = (centre[0] - (longueur/2.) * cos(angle) - (epaisseur/2.) * sin(angle),
            centre[1] + (epaisseur/2.) * cos(angle) - (longueur/2.) * sin(angle))
        BL = (centre[0] + (longueur/2.) * cos(angle) + (epaisseur/2.) * sin(angle),
            centre[1] - (epaisseur/2.) * cos(angle) + (longueur/2.) * sin(angle))
        BR = (centre[0] - (longueur/2.) * cos(angle) + (epaisseur/2.) * sin(angle),
            centre[1] - (epaisseur/2.) * cos(angle) - (longueur/2.) * sin(angle))

        self.liste_sommets=[UL,UR,BR,BL]



class Voiture:
    def __init__(self,x,y,route):
        self.x=x
        self.y=y
        self.vitesse=0
        self.route=route
        self.angle=0
        self.taille=(20,10)
        self.avancement=0
        self.route.liste_voitures.append(self)

        self.voiture_suivante=None
        self.distance_feu_circulation=inf
        self.distance_voiture_suivante=inf
        self.distance_obstacle=inf

        self.liste_parcours=[route]
        self.avancement_parcours=0

        self.liste_voitures_proches=[]

        self.polygone=0  # Polygon (type)
        self.polygone_vision=None     # Polygon (type)
        self.centre_masque_vision=(0,0)


        self.initialisation_proprietes()


    def initialisation_proprietes(self):
        self.acceleration_max=0.9
        self.vitesse_max=3
        self.lissage=4


        self.distance_min=15
        self.temps_reaction=40
        self.freinage=1




    def mettre_a_jour(self,dt):     # O(1)
        self.distance_obstacle=min(self.distance_voiture_suivante,self.distance_feu_circulation)

        self.angle=self.route.angle

        self.acceleration=self.acceleration_max*( (1-(self.vitesse/self.vitesse_max)**self.lissage) -(self.distance_desiree()))
        self.vitesse+=self.acceleration*dt

        if self.vitesse+self.acceleration*dt<0:
            self.vitesse=0
            self.avancement -= 0.5*self.vitesse*self.vitesse/self.acceleration
        else:
            self.vitesse+=self.acceleration*dt
            self.avancement+=self.vitesse+self.acceleration*(dt**2)/2

        self.x=self.route.point_debut.x+self.avancement*cos(self.route.angle)
        self.y=self.route.point_debut.y+self.avancement*sin(self.route.angle)
        self.pos=(self.x,self.y)

    def distance_desiree(self):
        if self.voiture_suivante!=None:

            return( ((self.distance_min+self.vitesse*self.temps_reaction+(self.vitesse*self.difference_vitesse_avec_suivant())/sqrt(2*self.acceleration_max*self.freinage))/self.distance_obstacle)**2 )
        else:

            return( ((self.distance_min+self.vitesse*self.temps_reaction+(self.vitesse**2)/sqrt(2*self.acceleration_max*self.freinage))/self.distance_obstacle)**2 )

    def difference_vitesse_avec_suivant(self):
        return(self.vitesse-self.voiture_suivante.vitesse)

    def route_franchie(self):   # O(1)
        if self.avancement>=self.route.longueur:
            return(True)






    def stocker_donnees(self,liste_vitesses,liste_accelerations,liste_vitesses_max):
        liste_vitesses.append(self.vitesse)
        liste_accelerations.append(self.acceleration)
        liste_vitesses_max.append(self.vitesse_max)



    def est_sortie(self,dimension):     # O(1)
        if self.x<0 or self.x>dimension[0] or self.y<0 or self.y>dimension[1]:
            return(True)
        else:
            return(False)



class Affichage:
    def __init__(self,simulation,temps_simulation,afficher):

        self.simulation=simulation
        self.afficher=afficher
        self.temps_simulation=temps_simulation

        self.surface_a_afficher=py.Surface((largeur_affichage,hauteur_affichage))
        self.surface_carte=py.Surface((largeur_carte,hauteur_carte))

        self.afficher_visions=False

        self.camera=Camera((0,0))







    def rotate_point(self,point, center, angle):
        """Rotation d'un point autour d'un centre donné avec un angle en radians."""
        rotated_x = math.cos(angle) * (point[0] - center[0]) - math.sin(angle) * (point[1] - center[1]) + center[0]
        rotated_y = math.sin(angle) * (point[0] - center[0]) + math.cos(angle) * (point[1] - center[1]) + center[1]
        return (rotated_x, rotated_y)

    def rotate_rectangle(self,rectangle, center, angle):
        """Rotation des sommets d'un rectangle autour d'un centre donné avec un angle en radians."""
        rotated_vertices = []
        for vertex in rectangle:
            rotated_vertex = self.rotate_point(vertex, center, angle)
            rotated_vertices.append(rotated_vertex)
        return rotated_vertices

    def mettre_a_jour_polygones(self,voiture):  # O(1)
        del voiture.polygone
        del voiture.polygone_vision

        x=voiture.x
        y=voiture.y
        longueur=voiture.taille[0]
        largeur=voiture.taille[1]
        facteur=15

        # polygone
        liste_sommets=[(x-longueur/2,y-largeur/2),(x+longueur/2,y-largeur/2),(x+longueur/2,y+largeur/2),(x-longueur/2,y+largeur/2)]
        sommets_orientes=self.rotate_rectangle(liste_sommets,(x,y),voiture.angle)

        voiture.polygone=Polygon(sommets_orientes)


        # polygone vision
        liste_sommets=[(x+1.2*longueur,y-largeur/9),(x+facteur*longueur/2,y-(facteur/4)*largeur),(x+facteur*longueur/2,y+(facteur/4)*largeur),(x+1.2*longueur,y+largeur/9)]
        sommets_orientes=self.rotate_rectangle(liste_sommets,(x,y),voiture.angle)

        voiture.polygone_vision=Polygon(sommets_orientes)


    def afficher_voitures(self):
        for voiture in self.simulation.liste_voitures:

            # polygone
            liste_sommets=list(voiture.polygone.exterior.coords)
            voiture.polygone=Polygon(liste_sommets)

            # polygone vision
            liste_sommets_vision=list(voiture.polygone_vision.exterior.coords)
            voiture.polygone_vision=Polygon(liste_sommets)

            if self.est_a_afficher(voiture.x,voiture.y)==True:
                py.draw.polygon(self.surface_carte,cyan,liste_sommets)
                if self.afficher_visions:
                    py.draw.polygon(self.surface_carte,rouge,liste_sommets_vision)





    def mettre_a_jour_voitures(self):
        nb_images_max=1000
        for voiture in self.simulation.liste_voitures:

            voiture.mettre_a_jour(self.simulation.dt)   # O(1)
            self.mettre_a_jour_polygones(voiture)   # O(1)

            self.verifier_suppression_voiture(voiture)  # O(1)
            self.verifier_changer_route(voiture)    # O(1)
            for w in range(1,floor((nb_images_max-2)*(1-(voiture.vitesse/voiture.vitesse_max)**2))+2+1):
                if self.simulation.nb_images%w==0:
                    self.mettre_a_jour_voitures_proches(voiture)    # O(liste_routes_proches*route.liste_voitures), on passe par une liste_voitures_proches pour optimiser si beaucoup de routes aux alentours
                    break
            self.mise_jour_distance_entourage(voiture)  # O(voiture.liste_voitures_proches+voiture.liste_voitures_proches)

        #self.mettre_a_jour_donnees()    # a changer de place !!

    def mettre_a_jour_voitures_proches(self,voiture):   # O(liste_routes_proches*route.liste_voitures)
        portee=30*(voiture.vitesse/voiture.vitesse_max)**2+70
        voiture.liste_voitures_proches=[]
        for route in voiture.route.liste_routes_proches:
            for voiture_bis in route.liste_voitures:
                if voiture_bis!=voiture and self.distance_pos((voiture.x,voiture.y),(voiture_bis.x,voiture_bis.y))<portee:
                    voiture.liste_voitures_proches.append(voiture_bis)
        # print(voiture.liste_voitures_proches)





    def mettre_a_jour_donnees(self):    # à refaire
        for voiture in self.simulation.liste_voitures:
            voiture.stocker_donnees(self.simulation.liste_vitesses,self.simulation.liste_accelerations,self.simulation.liste_vitesses_max)

        if self.simulation.liste_feux_circulation[0].etat=="rouge" and self.ok==False:
            self.simulation.liste_nb_voitures_passees.append(self.simulation.liste_compteurs[-1].compteur)
            self.simulation.liste_temps_feu_circulation.append(self.simulation.liste_feux_circulation[0].frequence)
            self.simulation.liste_feux_circulation[0].augmenter_frequence()
            self.simulation.liste_compteurs[-1].reinitialiser_compteur()
            self.ok=True

        if self.simulation.liste_feux_circulation[0].etat=="vert":
            self.ok=False





    def mise_jour_distance_entourage(self,voiture):     # O(voiture.liste_voitures_proches+voiture.liste_voitures_proches)
        self.mettre_a_jour_entourage_voiture(voiture)   # O(voiture.liste_voitures_proches)


        voiture.distance_feu_circulation=self.distance_avec_feu_circulation(voiture)    # O(voiture.route.liste_feux_circulation)

    def fonction_punition(self,voiture):
        # return((voiture.vitesse/voiture.vitesse_max)**2)
        return(0.5*(math.tanh(5*((voiture.vitesse/voiture.vitesse_max)-0.5))+1))

    def verifier_collision(self,voiture,voiture_bis):   # que si même route, O(1)
        if voiture.polygone.intersects(voiture_bis.polygone):
            self.simulation.nb_voitures_arrivees-=self.fonction_punition(voiture)*self.simulation.penalite_score_collision

        if voiture.polygone.intersects(voiture_bis.polygone) and voiture_bis.route==voiture.route:
            if voiture_bis.avancement>voiture.avancement:
                voiture.voiture_suivante=voiture_bis
                voiture.distance_voiture_suivante=self.distance_pos((voiture.x,voiture.y),(voiture_bis.x,voiture_bis.y))+.1     # +.1 pour éviter les divisions par 0 si la distance est nulle
                return(True)
        return(False)

    def verifier_changer_route(self,voiture):   # O(1)

        if voiture.route_franchie()==True:
            self.changer_route(voiture)

    def changer_route(self,voiture):
        voiture.route.liste_voitures.remove(voiture)

        # routes_adjacentes=self.routes_adjacentes(voiture.route)
        if voiture.route.liste_routes_adjacentes==[]:
            if voiture in self.simulation.liste_voitures:
                self.supprimer_voiture(voiture)
                self.simulation.nb_voitures_arrivees+=1
        else:
            voiture.liste_parcours.append(rd.choice(voiture.route.liste_routes_adjacentes))
            voiture.avancement_parcours+=1
            voiture.route=voiture.liste_parcours[voiture.avancement_parcours]
            voiture.avancement=0
            voiture.route.liste_voitures.append(voiture)



    def mettre_a_jour_entourage_voiture(self,voiture):     # s'occupe de voiture suivante et distance avec voiture suivante et collision, O(voiture.liste_voitures_proches)


        dico_voitures_potentielles={}
        for voiture_bis in voiture.liste_voitures_proches:

            # voiture regarde en face d'elle
            if voiture.polygone_vision.intersects(voiture_bis.polygone):
                dico_voitures_potentielles[voiture_bis]=self.distance_pos((voiture.x,voiture.y),(voiture_bis.x,voiture_bis.y))-voiture.taille[0]/2-voiture_bis.taille[0]/2

            # verifier collision avec autre voiture sur meme voie
            if self.verifier_collision(voiture,voiture_bis):
                return()    # voiture suivante trouvée


        # print(dico_voitures_potentielles)

        # recherche de la voiture et du minimum
        min=inf
        voiture_min=None
        for voiture_bis in dico_voitures_potentielles:
            if dico_voitures_potentielles[voiture_bis]<min:
                voiture_min=voiture_bis
                min=dico_voitures_potentielles[voiture_bis]
        voiture.distance_voiture_suivante,voiture.voiture_suivante=(min,voiture_min)





    def verifier_suppression_voiture(self,voiture):     # O(1)
        if voiture.est_sortie((largeur_carte,hauteur_carte))==True:
            self.supprimer_voiture(voiture)

    def supprimer_voiture(self,voiture):
        if voiture in self.simulation.liste_voitures:
            self.simulation.liste_voitures.remove(voiture)
        del voiture


    def afficher_routes(self):

        for route in self.simulation.liste_routes:
            for point in route.liste_points:
                if self.est_a_afficher(point.x,point.y)==True:
                    # self.ligne(route.point_debut.pos,route.point_fin.pos,epaisseur,noir)


                    gfx.aapolygon(self.surface_carte,route.liste_sommets,noir)
                    gfx.filled_polygon(self.surface_carte,route.liste_sommets,noir)


                    self.dessiner_fleche(route,1)
                    break

    def afficher_joints(self):
        for joint in self.simulation.liste_joints:
            if self.est_a_afficher(joint.x,joint.y)==True:
                self.cercle(joint.pos,epaisseur/2,noir)


    def afficher_feux_circulation(self):
        for feu_circulation in self.simulation.liste_feux_circulation:
            if self.est_a_afficher(feu_circulation.x,feu_circulation.y)==True:
                self.dessiner_feu_circulation(feu_circulation)

    def dessiner_feu_circulation(self,feu_circulation):
        if feu_circulation.etat=="rouge":
            couleur=rouge
        else:
            couleur=vert
        x = feu_circulation.x
        y = feu_circulation.y
        largeur_feu_circulation=epaisseur
        longueur_feu_circulation=10

        feu_circulation_surface=py.Surface((longueur_feu_circulation,largeur_feu_circulation),py.SRCALPHA)
        py.draw.rect(feu_circulation_surface,couleur,(0,0,longueur_feu_circulation,largeur_feu_circulation))
        feu_circulation_surface_orientee=py.transform.rotate(feu_circulation_surface, -math.degrees(feu_circulation.route.angle))
        # obtenir le rectangle englobant pour la surface de la voiture tournée
        rect_orientee = feu_circulation_surface_orientee.get_rect(center=(x+1, y+1))

        self.surface_carte.blit(feu_circulation_surface_orientee, rect_orientee)

    def afficher_generateurs(self):     # faire que l'actualisation des masques se fait à part et dans tout les cas !!!!!!!!!!!!!!!!!!!
        for generateur in self.simulation.liste_generateurs:

            # polygone
            liste_sommets=list(generateur.polygone.exterior.coords)
            generateur.polygone=Polygon(liste_sommets)

            # dessiner
            if self.est_a_afficher(generateur.x,generateur.y):
                py.draw.polygon(self.surface_carte,bleu_gris,liste_sommets)


    def afficher_compteurs(self):
        for compteur in self.simulation.liste_compteurs:
            if self.est_a_afficher(compteur.x,compteur.y):
                largeur_compteur=compteur.taille[0]
                hauteur_compteur=compteur.taille[1]

                compteur_surface=py.Surface((largeur_compteur, hauteur_compteur),py.SRCALPHA)
                py.draw.rect(compteur_surface,bleu_gris,(0,0,largeur_compteur,hauteur_compteur))
                compteur_surface_orientee=py.transform.rotate(compteur_surface, -math.degrees(compteur.route.angle))
                rect_orientee = compteur_surface_orientee.get_rect(center=(compteur.x+1, compteur.y+1))

                self.surface_carte.blit(compteur_surface_orientee, rect_orientee)

    def afficher_arrivees(self):
        for arrivee in self.simulation.liste_arrivees:
            if self.est_a_afficher(arrivee.x,arrivee.y):
                py.draw.rect(self.surface_carte,gris_clair,(arrivee.x-arrivee.taille[0]/2,arrivee.y-arrivee.taille[1]/2,arrivee.taille[0],arrivee.taille[1]))

    def afficher_intersections(self):
        pass
        # for intersection in self.simulation.liste_intersections:
        #     # if self.est_a_afficher(intersection.x,intersection.y):
        #     #     self.cercle((intersection.x_pondere,intersection.y_pondere),intersection.taille//2.5,noir)
        #
        #     for cardinal in liste_cardinaux:
        #         for i in range(len(intersection.dico_points[cardinal])):
        #             self.cercle(intersection.dico_points[cardinal][i].pos,10,cyan)


    def mettre_a_jour_feux_circulation(self):
        for feu_circulation in self.simulation.liste_feux_circulation:
            feu_circulation.attente+=self.simulation.dt
            feu_circulation.verifier_changement_etat()


    def distance_avec_feu_circulation(self,voiture):    # O(voiture.route.liste_feux_circulation)
        liste_distances=[]
        for feu_circulation_potentiel in voiture.route.liste_feux_circulation:
            if feu_circulation_potentiel.etat=="rouge" and feu_circulation_potentiel.placement_route*voiture.route.longueur>voiture.avancement:
                liste_distances.append(distance.euclidean(voiture.pos,feu_circulation_potentiel.pos))
        if liste_distances==[]:
            return(inf)
        return(min(liste_distances)-voiture.taille[0]/2)




    def mettre_a_jour_generateurs(self):
        for generateur in self.simulation.liste_generateurs:
            generateur.attente+=self.simulation.dt
            self.verifier_creation_generateur(generateur)
            generateur.mettre_a_jour_derniere_voiture()



    def verifier_creation_generateur(self,generateur):
        if generateur.doit_creer()==True:
            voiture=self.simulation.initialiser_voiture(*generateur.pos,generateur.route)
            voiture.mettre_a_jour(self.simulation.dt)
            self.mettre_a_jour_polygones(voiture)
            self.mettre_a_jour_voitures_proches(voiture)
            self.mise_jour_distance_entourage(voiture)
            generateur.derniere_voiture=voiture

        # if generateur.attente>=generateur.frequence:
            generateur.attente=0

    def mettre_a_jour_compteurs(self):
        for compteur in self.simulation.liste_compteurs:
            compteur.mettre_a_jour()





    # boucle de jeu
    def loop(self):

        py.init()
        # affichage
        if self.afficher==True:
            police=py.font.Font(None, 36)
            self.fenetre = py.display.set_mode((largeur_affichage, hauteur_affichage))
            py.display.set_caption("Simulation trafic routier :)")


        horloge=py.time.Clock()
        #horloge.tick_busy_loop()



        continuer=True
        while continuer:
            for event in py.event.get():
                if event.type==py.QUIT:
                    continuer=False
                if event.type == py.MOUSEBUTTONDOWN:
                    if event.button == 1:  # Clic gauche de la souris
                        # Obtenir la position du clic de souris
                        mouse_x, mouse_y = py.mouse.get_pos()
                        print("position:", self.camera.x+mouse_x,self.camera.y+mouse_y)
                if event.type==py.KEYDOWN:
                    if event.key == py.K_a:
                        self.simulation.dessiner=not self.simulation.dessiner
                    if event.key==py.K_ESCAPE:
                        self.simulation.stop=True
                        continuer=False
                    if event.key==py.K_q:
                        self.afficher_visions=not self.afficher_visions

            horloge.tick(self.simulation.fps)  # fps
            self.mettre_a_jour_camera()
            self.simulation.temps+=self.simulation.dt
            self.simulation.nb_images+=1
            if self.simulation.temps>=self.temps_simulation:
                continuer=False

            if self.afficher==True:
                self.mettre_a_jour_surface_a_afficher()

            self.mettre_a_jour_voitures()
            self.mettre_a_jour_generateurs()


            self.mettre_a_jour_feux_circulation()

            self.mettre_a_jour_compteurs()


            if self.afficher==True:
                self.dessiner_fond()
                if self.simulation.dessiner==True:
                    self.dessiner_elements()


                texte_surface1 = police.render("temps:"+str(round(self.simulation.temps,1)),True,noir)
                texte_surface2 = police.render("nombre d'itérations:"+str(self.simulation.nb_images),True,noir)
                texte_surface3 = police.render("fps:"+str(round(horloge.get_fps(),1)),True,noir)
                texte_surface4 = police.render("score:"+str(round(self.simulation.nb_voitures_arrivees,1)),True,noir)


                self.surface_a_afficher.blit(texte_surface1,(0, 0))
                self.surface_a_afficher.blit(texte_surface2,(0, 20))
                self.surface_a_afficher.blit(texte_surface3,(0, 40))
                self.surface_a_afficher.blit(texte_surface4,(0, 60))



                #self.fenetre.blit(self.surface_carte,(0,200))

                self.fenetre.blit(self.surface_a_afficher,(0,0))
                py.display.flip()   # mise à jour de l'affichage

        py.quit()

        #plt.scatter(self.simulation.liste_vitesses,self.simulation.liste_accelerations)
        #plt.scatter(self.simulation.liste_densites,self.simulation.liste_vitesses_moyennes_normalisees)
        # plt.scatter(self.simulation.liste_temps_feu_circulation,self.simulation.liste_nb_voitures_passees)

        # if tracer==True:
        #     plt.show()

    def liste_sommets_adjacents(self,nom_point):
        liste_sommets=[]
        for j in range(len(self.simulation.matrice_adjacence)):
            if self.simulation.matrice_adjacence[nom_point][j]!=0:
                liste_sommets.append(j)
        return(liste_sommets)

    def liste_noms_points(self):
        liste_nom_points=[]
        for i in range(self.matrice_adjacence):
            for j in range(self.matrice_adjacence):
                if self.matrice_adjacence[i][j]!=0:
                    if not i in liste_nom_points:
                        liste_nom_points.append(i)
                    if not j in liste_nom_points:
                        liste_nom_points.append(j)
        return(liste_noms_points)

    def cout(self,chemin=[]):
        cout=0
        for arete in chemin:
            cout+=self.simulation.matrice_adjacence[arete[0]][arete[1]]
        return(cout)

    def plus_court_chemin(self,nom_point_debut,nom_point_fin):  # A FINIR OH!!!!!!!!!!
        dico_points={}
        dico_chemins={}
        liste_noms_points=liste_noms_points()
        for nom_point in liste_noms_points:
            dico_points[nom_point]=inf
            dico_chemins[nom_point]=[]
        a_traiter=[nom_point_debut]
        while a_traiter!=[]:
            x=a_traiter[0]
            a_traiter.append(self.liste_sommets_adjacents(x))
            dico_chemins[x].append(x)
            cout_x=self.cout(dico_chemins[x])

    def distance_pos(self,pos1,pos2):
        x1,y1=pos1
        x2,y2=pos2
        return(sqrt(abs(x2-x1)**2+abs(y2-y1)**2))



    def mettre_a_jour_camera(self):
        keys = py.key.get_pressed()
        if keys[py.K_LEFT]:
            if self.camera.x-self.camera.vitesse>=0:
                self.camera.x -=self.camera.vitesse
        if keys[py.K_RIGHT]:
            if self.camera.x+largeur_affichage+self.camera.vitesse<=largeur_carte:
                self.camera.x +=self.camera.vitesse
        if keys[py.K_UP]:
            if self.camera.y-self.camera.vitesse>=0:
                self.camera.y -=self.camera.vitesse
        if keys[py.K_DOWN]:
            if self.camera.y+hauteur_affichage+self.camera.vitesse<=hauteur_carte:
                self.camera.y +=self.camera.vitesse







    def dessiner_fond(self):
        self.surface_a_afficher.fill(blanc)    # arrière plan

        #self.surface_carte.blit(self.surface_a_afficher,(0,0))

        self.dessiner_grille(100, (200,200,200))
    def dessiner_elements(self):
        self.afficher_joints()
        self.afficher_routes()
        self.afficher_intersections()
        self.afficher_voitures()
        self.afficher_feux_circulation()
        self.afficher_generateurs()
        self.afficher_compteurs()
        self.afficher_arrivees()





    def cercle(self,pos:(),r,couleur):
        py.draw.circle(self.surface_carte,couleur,pos,r)


    def ligne(self,pos1,pos2,largeur,couleur):
        #py.draw.line(self.surface_carte, couleur,pos1,pos2,largeur)

        centre=(pos1[0]+pos2[0])/2,(pos1[1]+pos2[1])/2
        longueur=distance.euclidean(pos1,pos2)  # Total longueur of line
        angle = math.atan2(pos1[1] - pos2[1], pos1[0] - pos2[0])

        UL = (centre[0] + (longueur/2.) * cos(angle) - (largeur/2.) * sin(angle),
            centre[1] + (largeur/2.) * cos(angle) + (longueur/2.) * sin(angle))
        UR = (centre[0] - (longueur/2.) * cos(angle) - (largeur/2.) * sin(angle),
            centre[1] + (largeur/2.) * cos(angle) - (longueur/2.) * sin(angle))
        BL = (centre[0] + (longueur/2.) * cos(angle) + (largeur/2.) * sin(angle),
            centre[1] - (largeur/2.) * cos(angle) + (longueur/2.) * sin(angle))
        BR = (centre[0] - (longueur/2.) * cos(angle) + (largeur/2.) * sin(angle),
            centre[1] - (largeur/2.) * cos(angle) - (longueur/2.) * sin(angle))

        gfx.aapolygon(self.surface_carte, (UL, UR, BR, BL),couleur)
        gfx.filled_polygon(self.surface_carte, (UL, UR, BR, BL),couleur)



    def dessiner_fleche(self,route,epaisseur_fleche):
        intervalle_fleche=100
        nb_fleches=int(distance.euclidean(route.point_debut.pos,route.point_fin.pos)/intervalle_fleche)
        couleur=gris_clair
        longueur_fleche=10

        x0,y0=route.point_debut.pos


        for i in range(1,nb_fleches+1):
            x = x0 + i * intervalle_fleche*cos(route.angle)
            y = y0 + i * intervalle_fleche*sin(route.angle)

            # Calcul de l'angle de la flèche
            angle_fleche = route.angle + math.pi

            # Calcul des coordonnées des extrémités de la flèche
            x1 = x + longueur_fleche * math.cos(angle_fleche - math.pi / 6)
            y1 = y + longueur_fleche * math.sin(angle_fleche - math.pi / 6)
            x2 = x + longueur_fleche * math.cos(angle_fleche + math.pi / 6)
            y2 = y + longueur_fleche * math.sin(angle_fleche + math.pi / 6)

            # Dessin de la flèche
            py.draw.line(self.surface_carte, couleur, (x1, y1), (x, y), epaisseur_fleche)
            py.draw.line(self.surface_carte, couleur, (x2, y2), (x, y), epaisseur_fleche)





    def dessiner_grille(self,taille_case,couleur:()):
        for x in range(0,largeur_carte,taille_case):
            if x+marge>=self.camera.x and x+marge<=self.camera.x+largeur_affichage:
                py.draw.line(self.surface_carte,(200,200,200),(x,0),(x,hauteur_carte))
        for y in range(0, hauteur_carte, taille_case):
            if y+marge>=self.camera.y and y+marge<=self.camera.y+hauteur_affichage:
                py.draw.line(self.surface_carte,(200,200,200),(0,y),(largeur_carte,y))

    def est_a_afficher(self,x,y):
        if x+marge>=self.camera.x+largeur_affichage or x-marge<=self.camera.x or y+marge>=self.camera.y+hauteur_affichage or y-marge<=self.camera.y:
            return(False)
        else:
            return(True)

    def mettre_a_jour_surface_a_afficher(self):
        portion_a_afficher=py.Rect(self.camera.x,self.camera.y,largeur_affichage,hauteur_affichage)
        self.surface_a_afficher=self.surface_carte.subsurface(portion_a_afficher)












class Algorithme():
    def __init__(self,nb_individus,nb_generations,temps_simulation,fps):
        self.nb_individus=nb_individus
        self.temps_simulation=temps_simulation
        self.nb_generations=nb_generations
        self.fps=fps

        if reprendre_simulation==False:
            self.liste_individus=[]
            self.numero_generation=0
            self.numero_individu=1
        else:
            while len(self.liste_individus)<self.nb_individus:
                self.liste_individus.append(Individu([rd.choice(self.config["liste_routes"]),rd.uniform(0.1,0.9),rd.uniform(1,10),rd.choice(["rouge","vert"])]))

        self.frequence_min=1
        self.frequence_max=10
        self.placement_route_min=0.1
        self.placement_route_max=0.9

        self.config=None



        self.simulation=Simulation(self.fps)




        self.simulation.afficher=True
        self.simulation.dessiner=False



    def controle(self):
        if not reprendre_simulation==True:
            self.creer_premiere_population()
        while self.numero_generation<=self.nb_generations:
            print("GENERATION :",self.numero_generation)
            for individu in self.liste_individus:
                # print(individu.liste_feux_circulation)
                self.initialiser_individu(individu.liste_feux_circulation)
                if self.simulation.stop==True:
                    break
                individu.score=self.simulation.nb_voitures_arrivees
                print("INDIVIDU",self.liste_individus.index(individu)+1,":",individu.score)
            self.numero_generation+=1
            if self.simulation.stop==True:
                break

            # print("a",self.liste_individus)

            liste_individus_suivants=[]
            self.liste_individus.sort(key=self.regarder_score)
            indice_separation=len(self.liste_individus)//2
            for i in range(indice_separation-1,len(self.liste_individus)-1,1):
                liste_individus_suivants.append(self.creer_individu(self.liste_individus[i],self.liste_individus[i+1]))
                liste_individus_suivants.append(self.creer_individu(self.liste_individus[i],self.liste_individus[i+1]))
            self.liste_individus[:]=liste_individus_suivants[:]
            # print("p",self.liste_individus)
            # print(self.liste_individus)


    def regarder_score(self,individu):
        return(individu.score)

    def initialiser_individu(self,liste_feux_circulation):   # config={"dico_points":{},"liste_routes":[],"liste_feux_circulation":[],"liste_generateurs":[]}

        # if "simulation" in locals():
        #     del self.simulation
        if "affichage" in locals():
            del self.affichage

        # print("liste_feux_circulation",liste_feux_circulation)

        self.simulation.supprimer_elements_simulation()

        self.simulation.initialiser_dico_points(self.config["dico_points"])
        self.simulation.initialiser_liste_routes(self.config["liste_routes"])
        self.simulation.initialiser_liste_feux_circulation(liste_feux_circulation)
        self.simulation.initialiser_liste_generateurs(self.config["liste_generateurs"])

        self.simulation.initialiser_matrice_adjacence()
        self.simulation.initialiser_reste()


        self.affichage=Affichage(self.simulation,self.temps_simulation,self.simulation.afficher)
        self.affichage.loop()




    def creer_individu(self,parent1,parent2):  # liste_parent1=[((nom_point,nom_point),placement_route,frequence,etat_initial),...], parent1 est un individu
        liste_parent1=parent1.liste_feux_circulation
        liste_parent2=parent2.liste_feux_circulation
        if rd.random()<=1/2:
            nb_feux_circulation=len(liste_parent1)
        else:
            nb_feux_circulation=len(liste_parent2)
        coef_mutation=0.25

        indice_changement=rd.randint(0,4)
        liste_enfant=[]

        if rd.random()<=coef_mutation:   # ajout d'un feu
            liste_enfant.append([rd.choice(self.config["liste_routes"]),rd.uniform(0.1,0.9),rd.uniform(1,10),rd.choice(["rouge","vert"])])
        for i in range(nb_feux_circulation):
            feu_circulation=[]


            if liste_enfant==[] or rd.random()>coef_mutation:    # supprimer feu ou non

                for j in range(len(liste_parent1[0])):
                    if j<=indice_changement:
                        gene=liste_parent1[min(i,len(liste_parent1)-1)][j]
                    else:
                        gene=liste_parent2[min(i,len(liste_parent2)-1)][j]



                    if j==0:
                        if rd.random()<=coef_mutation/2:
                            gene=rd.choice(self.config["liste_routes"])
                    elif j==1:
                        gene=clamp(rd.uniform(gene-(self.placement_route_max-self.placement_route_min)*coef_mutation,gene+(self.placement_route_max-self.placement_route_min)*coef_mutation),0.1,0.9)

                    elif j==2:
                        gene=clamp(rd.uniform(gene-(self.frequence_max-self.frequence_min)*coef_mutation,gene+(self.frequence_max-self.frequence_min)*coef_mutation),self.frequence_min,self.frequence_max)
                    elif gene=="rouge":
                        if rd.random()<=coef_mutation:
                            gene="vert"
                    elif gene=="vert":
                        if rd.random()<=coef_mutation:
                            gene="rouge"

                    # print(gene)
                    feu_circulation.append(gene)
            # print("FEU_CIRCULATION",feu_circulation)
            if feu_circulation!=[]:
                liste_enfant.append(feu_circulation)
        return(Individu(liste_enfant))







    def creer_premiere_population(self):
        for _ in range(self.nb_individus):
            nb_feux_circulation=rd.randint(1,10)
            individu=Individu([[rd.choice(self.config["liste_routes"]),rd.uniform(0.1,0.9),rd.uniform(1,10),rd.choice(["rouge","vert"])] for __ in range(nb_feux_circulation)])
            self.liste_individus.append(individu)


class Individu:     # contient liste_feux_circulation
    def __init__(self,liste_feux_circulation):
        self.liste_feux_circulation=liste_feux_circulation
        self.nb_feux_circulation=len(liste_feux_circulation)
        self.score=inf  # sera le nombre de voitures arrivées




reprendre_simulation=False

temps_simulation=45
fps=800
nb_individus=20
nb_generations=10

if reprendre_simulation==False:
    algorithme=Algorithme(nb_individus,nb_generations,temps_simulation,fps)


algorithme.config={"dico_points":{0:(500,100),1:(700,450),2:(1000,700),3:(550,550),4:(150,500),5:(50,600),6:(861,45),7:(1000,1000),8:(150,250),9:(678,52),10:(600,100),11:(650,800),12:(1000,550),13:(340,745),14:(180,950),15:(0,650),16:(790,1070),17:(890,800),18:(480,610),19:(1203,496)},"liste_routes":[(0,1),(1,2),(2,7),(3,4),(6,1),(4,5),(1,3),(4,8),(9,1),(1,11),(11,13),(13,14),(13,15),(13,16),(2,17),(18,11)],"liste_generateurs":[(0,(0,1),1.6),(6,(6,1),1.5),(9,(9,1),1.5),(18,(18,11),2)]}


debut=time.time()
algorithme.controle()
fin=time.time()
print("durée de l'execution :"+str(fin-debut)+"s")

# debut=time.time()
# algorithme.initialiser_individu([])
# fin=time.time()
# print("durée de l'execution :"+str(fin-debut)+"s")
# print("nombre de voitures arrivées:"+str(algorithme.simulation.nb_voitures_arrivees))

class Flag:
    def __init__(self):
        pass


# config=algorithme.creer_simulation_aleatoire({"nb_points":5,"espacement":500,"nb_routes":3,"densité_routes":0.3,"densité_feu_circulation":0.5,"nb_generateurs":3})


"""    def creer_simulation_aleatoire(self,parametres={}):    # {"nb_points":5,"espacement":500,"nb_routes":3,"densité_routes":0.3,"densité_feu_circulation":0.5,"nb_generateurs":3}

        # placer points
        liste_coordonnees=[(rd.randint(0,1920*facteur),rd.randint(0,1080*facteur))]
        for i in range(parametres["nb_points"]-1):

            ok=False
            while not ok:
                nb_ok=0
                nouv_coordonnees=(rd.randint(0,1920*facteur),rd.randint(0,1080*facteur))
                for coordonnees in liste_coordonnees:
                    if distance.euclidean(coordonnees,nouv_coordonnees)>parametres["espacement"]:
                        nb_ok+=1
                if nb_ok==len(liste_coordonnees):
                    ok=True
            liste_coordonnees.append(nouv_coordonnees)

        dico_points_i={}
        for i in range(parametres["nb_points"]):
            dico_points_i[i]=liste_coordonnees[i]



        # placer routes
        liste_routes_i=[]
        for i in range(parametres["nb_points"]):
            for j in range(parametres["nb_points"]):
                if i!=j and not (i,j) in liste_routes_i and not (j,i) in liste_routes_i:
                    if rd.random()<=parametres["densité_routes"]:
                        liste_routes_i.append((i,j))

        liste_feux_circulation_i=[]
        for route in liste_routes_i:
            if rd.random()<=parametres["densité_feu_circulation"]:
                liste_feux_circulation_i.append((route,rd.uniform(0.2,0.8),rd.randint(30,60),"vert"))

        liste_generateurs_i=[]
        for w in range(parametres["nb_generateurs"]):
            route=rd.choice(liste_routes_i)
            liste_generateurs_i.append((route[0],route,rd.randint(2,5)))


        config={}
        config["dico_points"]=dico_points_i
        config["liste_routes"]=liste_routes_i
        config["liste_feux_circulation"]=liste_feux_circulation_i
        config["liste_generateurs"]=liste_generateurs_i
        return(config)"""


# réseau 1: classique

"""simulation.initialiser_dico_points( {0:(500,100),1:(700,450),2:(1000,700),3:(550,550),4:(150,500),5:(50,600),6:(861,45),7:(1000,1000),8:(150,250),9:(678,52),10:(600,100),11:(650,800),12:(1000,550),13:(340,745),14:(180,950),15:(0,650),16:(790,1070),17:(890,800),18:(480,610),19:(1203,496)} )    #{nom_point:(x,y)}


simulation.initialiser_liste_routes( [(0,1),(1,2),(2,7),(3,4),(6,1),(4,5),(1,3),(4,8),(9,1),(1,11),(11,13),(13,14),(13,15),(13,16),(2,17),(18,11)] )    #[(nom_point,nom_point)]
# simulation.initialiser_liste_feux_circulation( [((0,1),0.9,5,"rouge"),((6,1),0.9,5,"vert")] )   #[((nom_point,nom_point),placement_route,frequence,etat]  # maintenant traité comme des listes de listes

simulation.initialiser_voiture(200,100,simulation.liste_routes[0])
simulation.initialiser_liste_generateurs( [(0,(0,1),1.6),(6,(6,1),1.5),(9,(9,1),1.5),(18,(18,11),2)] )
simulation.initialiser_liste_compteurs( [((0,1),0.5)] )"""


# réseau 2: tests feu circulation
"""simulation.initialiser_dico_points( {0:(10,10),1:(3000,3000)} )    #{nom_point:(x,y)}

simulation.initialiser_liste_routes( [(0,1)] )    #[(nom_point,nom_point)]

simulation.initialiser_voiture(200,100,simulation.liste_routes[0])
simulation.initialiser_liste_generateurs( [(0,(0,1),0.7)] )
simulation.initialiser_liste_feux_circulation( [((0,1),0.33,2,"rouge")] )
simulation.initialiser_liste_compteurs( [((0,1),0.05),((0,1),0.4)] )"""

# réseau 3: carrefour
"""simulation.initialiser_dico_points( {0:(800,200),1:(800,600),2:(1200,600),3:(400,600),4:(800,1000),5:(900,200),6:(1200,700),7:(700,1000),8:(400,700)} )

simulation.initialiser_liste_routes( [(0,1),(1,0),(2,1),(1,2),(3,1),(4,1),(1,4),(1,3)] )
simulation.initialiser_liste_feux_circulation( [] )

simulation.initialiser_voiture(200,100,simulation.liste_routes[0])
simulation.initialiser_liste_generateurs( [(0,(0,1),2),(2,(2,1),2.2),(3,(3,1),2.4),(4,(4,1),2.5)] )
simulation.initialiser_liste_compteurs( [((0,1),0.5)] )"""





def dessiner_graphe_associe():
    graphe=nx.MultiDiGraph()

    liste_noeuds=[]
    for i in range(simulation.taille_matrice_adjacence):
        for j in range(simulation.taille_matrice_adjacence):
            if simulation.matrice_adjacence[i][j]==1:
                if not i in liste_noeuds:
                    liste_noeuds.append(i)
                    graphe.add_node(i)
                if not j in liste_noeuds:
                    liste_noeuds.append(j)
                    graphe.add_node(j)

    for i in range(simulation.taille_matrice_adjacence):
        for j in range(simulation.taille_matrice_adjacence):
            if simulation.matrice_adjacence[i][j]==1:
                graphe.add_edge(i,j,poids=round(simulation.route_associee(i,j).longueur,1))


    positions_noeuds={}
    for i in liste_noeuds:
        positions_noeuds[i]=(simulation.pos_depuis_nom_point(i)[0],-simulation.pos_depuis_nom_point(i)[1])

    nx.draw_networkx(graphe,pos=positions_noeuds, with_labels=True, node_size=500, node_color='lightblue',edge_color='gray')
    noms_poids=nx.get_edge_attributes(graphe,"poids")
    #nx.draw_networkx_edge_labels(graphe, pos=positions_noeuds, edge_labels=noms_poids)
    plt.show()



"""afficher_points_inutiles=False  # pour debugger (à retravailler)
simulation.initialiser_matrice_adjacence()
# dessiner_graphe_associe()
simulation.initialiser_reste()
affichage=Affichage(simulation,True)
affichage.loop()"""



# faire en sorte que les donnees se renouvellent avec le temps
# extraire données
# chaque voiture ne peut adapter sa vitesse que en fonction de voiture sur la meme route
#   -> elles doivent savoir ou aller: plus court chemin
# faire "maisons" pour arrivés des voitures
# caractériser (et quantifier) la capacité d'un feu de circulation: nb voitures passées au vert
# avoir un pdv macroscopique: sur chaque route

# faire que l'init d'une voiture se fasse: initialiser_voiture(avancement,route) et non via sa position

# faire doubles voies!!
# finir plus court chemin
# faire que compteur ait un nom "(nom_point,nom_point)" et qu'il stock ses données dans une liste à lui

# faire algorithme A* (Dijkstra heuristique)


# Edmonds Karp Algorithm pour probleme du flot maximal

# distance.euclidean très couteuse en complexité?????
# faire courbe bézier pour TOUS LES VIRAGES

# faire que distance seuil pour voitures proches s'adapte selon la vitesse
# finir de faire qu'il y a actualisation des polygones des objets qu'on dessine
# optimiser le dessinement


"""Algorithme"""
# faire qu'il est simple de faire une simulation lambda sans algorithme
# faire qu'il est simple de reprendre une simulation fini