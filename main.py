import numpy as np
import cv2
from scipy.signal import convolve2d

class Detection_points_interets:

    def __init__(self, path):

        image_originelle = cv2.imread(path, cv2.IMREAD_GRAYSCALE)

        hauteur, largeur = image_originelle.shape
        self.dimensions_image = {"hauteur":hauteur, "largeur":largeur}

        self.valeur_pixel_supprime = -np.inf

        self.metriques_physiques()

        image_changement_de_valeur = self.changement_des_valeurs_de_l_image(image_originelle)
        image_avec_pixels_connus_et_inconnus = self.suppression_des_zones_autours_des_obstacles(image_changement_de_valeur)
        image_avec_pixels_connus_accessibles = self.suppression_des_pixels_connus_non_accessibles(image_avec_pixels_connus_et_inconnus)
        image_avec_pixels_connus_et_inconnus_accessibles = self.suppression_des_pixels_inconnus_non_accessibles(image_avec_pixels_connus_accessibles)
        image_des_zones_a_explorer = self.application_du_filtre_moyenneur(image_avec_pixels_connus_et_inconnus_accessibles)
        image_des_zones_a_explorer_moyennee = self.application_du_filtre_moyenneur(image_des_zones_a_explorer)
        image_des_zones_a_explorer_moyennee_normalisee = self.normalisation_de_image(image_des_zones_a_explorer_moyennee)

        
    
    def metriques_physiques(self):

        ###### Metriques pour les filtres ######

        taille_pixel_en_realite = {"hauteur":0.1, "largueur":0.1}
        taille_robot_en_realite = {"hauteur":0.3, "largueur":0.3}

        self.taille_robot_en_pixel = {
            "hauteur":int(taille_robot_en_realite["hauteur"]/taille_pixel_en_realite["hauteur"]),
            "largueur":int(taille_robot_en_realite["largueur"]/taille_pixel_en_realite["largueur"])
        }

        self.matrice_zone_robot = np.ones((self.taille_robot_en_pixel["hauteur"], self.taille_robot_en_pixel["largueur"]), dtype=np.float32)


    def changement_des_valeurs_de_l_image(self, image):

        ###### Changement de valeurs ######

        image_changement_de_valeur = np.copy(image)

        self.valeur_pixel_occupe = {"image_origine":128, "image_voulue":-100}
        self.valeur_pixel_connu = {"image_origine":255, "image_voulue":0}
        self.valeur_pixel_inconnu = {"image_origine":0, "image_voulue":100}

        image_changement_de_valeur[image==self.valeur_pixel_occupe["image_origine"]] = self.valeur_pixel_occupe["image_voulue"]
        image_changement_de_valeur[image==self.valeur_pixel_connu["image_origine"]] = self.valeur_pixel_connu["image_voulue"]
        image_changement_de_valeur[image==self.valeur_pixel_inconnu["image_origine"]] = self.valeur_pixel_inconnu["image_voulue"]

        return image_changement_de_valeur
    

    def suppression_des_zones_autours_des_obstacles(self, image):

        ###### Suppression zones ou le robot ne peut physiquement pas aller ######

        image_avec_pixels_connus_et_inconnus_accessibles = np.copy(image)

        filtre_moyenneur_autour_robot = self.matrice_zone_robot/(self.taille_robot_en_pixel["hauteur"]*self.taille_robot_en_pixel["largueur"])

        masque_pixels_occupes = (image==self.valeur_pixel_occupe["image_voulue"])
        image_avec_que_les_pixels_occupes = image*masque_pixels_occupes

        image_avec_zones_autour_des_pixels_occupes = convolve2d(image_avec_que_les_pixels_occupes, filtre_moyenneur_autour_robot, mode='same', boundary='fill', fillvalue=0)

        masque_des_zones_autour_des_pixels_occupes = (image_avec_zones_autour_des_pixels_occupes!=0)
        image_avec_pixels_connus_et_inconnus_accessibles[masque_des_zones_autour_des_pixels_occupes] = self.valeur_pixel_supprime

        return image_avec_pixels_connus_et_inconnus_accessibles
    
    def suppression_des_pixels_connus_non_accessibles(self, image):

        image_avec_pixels_connus_accessibles = np.copy(image)

        return image_avec_pixels_connus_accessibles
    
    def suppression_des_pixels_inconnus_non_accessibles(self, image):


        image_avec_pixels_connus_et_inconnus_accessibles = np.copy(image)

        filtre_comptabiliseur = np.array([
            [0.5,0.5,0.5,0.5,0.5],
            [0.5,1.0,1.0,1.0,0.5],
            [0.5,1.0,0.0,1.0,0.5],
            [0.5,1.0,1.0,1.0,0.5],
            [0.5,0.5,0.5,0.5,0.5],
        ])

        masque_des_pixels_connus = (image == self.valeur_pixel_connu["image_voulue"])

        nombres_de_pixels_connus_aux_alentours = convolve2d(masque_des_pixels_connus, filtre_comptabiliseur, mode='same', boundary='fill', fillvalue=0)

        masque_presence_des_pixels_connus_aux_alentours = (nombres_de_pixels_connus_aux_alentours>1)

        image_avec_pixels_connus_et_inconnus_accessibles[~masque_presence_des_pixels_connus_aux_alentours] = self.valeur_pixel_supprime

        # multiplication avec le nombre de pixels connus aux alentours pour identifier les concentrations
        image_avec_pixels_connus_et_inconnus_accessibles[masque_presence_des_pixels_connus_aux_alentours] *= nombres_de_pixels_connus_aux_alentours

        return image_avec_pixels_connus_et_inconnus_accessibles


    def application_du_filtre_moyenneur(self, image):

        ###### Application d'un filtre moyenneur ######

        image_moyenne_excluant_les_pixels_supprimes = np.copy(image)
        masque_pixels_supprimes = (image_moyenne_excluant_les_pixels_supprimes==self.valeur_pixel_supprime)

        # suppression des pixels supprimes
        image_moyenne_excluant_les_pixels_supprimes[masque_pixels_supprimes] = 0

        taille_du_filtre = (3,3)
        filtre_moyenneur = np.ones(taille_du_filtre, dtype=np.float32)/9

        image_moyenne_sans_pixels_supprimes = convolve2d(image_moyenne_excluant_les_pixels_supprimes, filtre_moyenneur, mode='same', boundary='fill', fillvalue=self.valeur_pixel_inconnu["image_voulue"])
        
        # restauration des pixels supprimes
        image_moyenne_sans_pixels_supprimes[masque_pixels_supprimes] = self.valeur_pixel_supprime

        return image_moyenne_sans_pixels_supprimes
    
    def normalisation_de_image(self, image):

        image_normalisee = np.copy(image)
        masque_pixels_supprimes = (image_normalisee==self.valeur_pixel_supprime)
        image_normalisee[masque_pixels_supprimes] = 0

        minimum = np.min(image_normalisee)
        maximum = np.max(image_normalisee)

        image_normalisee = (image_normalisee - minimum)*(1/(maximum-minimum))

        return image_normalisee
    

if "__main__" == __name__:
    path_image = "mon image.png"
    detection_des_points_d_interets = Detection_points_interets(path_image)



