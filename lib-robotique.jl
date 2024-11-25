using LinearAlgebra #pour la pseudo-inverse
using Plots #pour les affichages

mutable struct DH
    α::Array{Float64,1}
    d::Array{Float64,1}
    r::Array{Float64,1}
    m::Array{Float64,1}
    c::Array{Float64,2}
end


function CreateRobotKukaLwr()
    α = [0.0, pi / 2, -pi / 2, -pi / 2, pi / 2, pi / 2, -pi / 2]
    d = zeros(7)
    r = [0.3105, 0.0, 0.4, 0.0, 0.39, 0.0, 0.078]
    θ = zeros(7)
    m = [2.7, 2.7, 2.7, 2.7, 2.7, 2.7, 0.3]
    c = zeros(4, 7)
    c[2, :] = [-8.70e-3, 8.7e-3, 8.7e-3, -8.7e-3, -8.2e-3, -7.6e-3, 0.0]
    c[3, :] = [-1.461e-2, 1.461e-2, -1.461e-2, 1.461e-2, -3.48e-2, 1.363e-3, 0.0]
    c[4, :] = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    robot_kuka = DH(α, d, r, m, c)
    return robot_kuka
end

function MGD(θ, robot)
    N = length(robot.α)
    T = zeros(4, 4)
    Tp = Matrix(I, 4, 4)
    for i = 1:N
        cθ = cos(θ[i])
        sθ = sin(θ[i])
        d = robot.d[i]
        sα = sin(robot.α[i])
        cα = cos(robot.α[i])
        r = robot.r[i]
        T = Tp * [cθ -sθ 0.0 d;
            cα*sθ cα*cθ -sα -r*sα;
            sα*sθ sα*cθ cα r*cα;
            0.0 0.0 0.0 1.0]
        Tp = T
    end
    return T
end

function Jacobian(θ, robot, p0)
    N = length(robot.α)
    T = zeros(4, 4)
    l = zeros(3, N)
    z = zeros(3, N)
    Tp = Matrix(I, 4, 4)
    J = zeros(6, N)
    for i = 1:N
        cθ = cos(θ[i])
        sθ = sin(θ[i])
        d = robot.d[i]
        sα = sin(robot.α[i])
        cα = cos(robot.α[i])
        r = robot.r[i]
        T = Tp * [cθ -sθ 0.0 d;
            cα*sθ cα*cθ -sα -r*sα;
            sα*sθ sα*cθ cα r*cα;
            0.0 0.0 0.0 1.0]
        l[:, i] = p0[:] - T[1:3, 4]
        z[:, i] = T[1:3, 3]
        J[1:3, i] = cross(z[:, i], l[:, i])
        J[4:6, i] = z[:, i]
        Tp = T
    end
    return J
end


function CoM(θ, robot)
    """ test """
    N = length(robot.α)
    M = sum(robot.m[1:N])
    T = zeros(4, 4)
    CoM = zeros(4, 1)
    CoMp = zeros(4, 1)
    Tp = Matrix(I, 4, 4)
    for i = 1:N
        cθ = cos(θ[i])
        sθ = sin(θ[i])
        d = robot.d[i]
        sα = sin(robot.α[i])
        cα = cos(robot.α[i])
        r = robot.r[i]
        T = Tp * [cθ -sθ 0.0 d;
            cα*sθ cα*cθ -sα -r*sα;
            sα*sθ sα*cθ cα r*cα;
            0.0 0.0 0.0 1.0]
        CoM[:] = (robot.m[i] / M) .* T * robot.c[:, i] + CoMp
        CoMp = CoM
        Tp = T
    end
    return CoM
end

function JacobianCoM(θ, robot, CoM0)
    N = length(robot.α)
    M = sum(robot.m[1:N])
    CoM = zeros(4, 1)
    CoMp = zeros(4, 1)
    T = zeros(4, 4)
    l = zeros(3, N)
    z = zeros(3, N)
    Tp = Matrix(I, 4, 4)
    CoMo = zeros(4, 1)
    Jcom = zeros(3, N)
    for i = 1:N
        cθ = cos(θ[i])
        sθ = sin(θ[i])
        d = robot.d[i]
        sα = sin(robot.α[i])
        cα = cos(robot.α[i])
        r = robot.r[i]
        T = Tp * [cθ -sθ 0.0 d;
            cα*sθ cα*cθ -sα -r*sα;
            sα*sθ sα*cθ cα r*cα;
            0.0 0.0 0.0 1.0]
        CoMo[:] = (robot.m[i] / M) .* T * robot.c[:, i] + CoMp
        CoMp = CoMo
        l[:, i] = CoM0[1:3] - CoMo[1:3]
        z[:, i] = T[1:3, 3]
        Jcom[1:3, i] = cross(z[:, i], l[:, i])
        Tp = T
    end
    return Jcom
end


#-----------------------
#θ=[0.143019, -0.109465, -0.011994, -1.1788, -0.154233, 0.93555, 0.264868];
#-----------------------


function RTL2R(ϕ, θ, ψ)
    A = [cos(θ)*cos(ϕ) -sin(ϕ)*cos(ψ)+cos(ϕ)*sin(θ)*sin(ψ) sin(ψ)*sin(ϕ)+cos(ϕ)*sin(θ)*cos(ψ)
        cos(θ)*sin(ϕ) cos(ϕ)*cos(ψ)+sin(θ)*sin(ψ)*sin(ϕ) -cos(ϕ)*sin(ψ)+sin(θ)*sin(ϕ)*cos(ψ)
        -sin(θ) cos(θ)*sin(ψ) cos(θ)*cos(ψ)]
    return A
end

function R2RTL(R)
    θ = -asin(R[3, 1])
    ϕ = atan(R[2, 1], R[1, 1])
    ψ = atan(R[3, 2], R[3, 3])
    return ϕ, θ, ψ
end

function B0(ϕ, θ, ψ)
    R = [0 -sin(ϕ) cos(θ)*cos(ϕ)
        0 cos(ϕ) cos(θ)*sin(ϕ)
        1 0 -sin(θ)]
    return R
end

"""  
    Fonction MCI en translation pour un robot à 7DDL qui déplace uniquement la position (translation) de l'effecteur final.

    Arguments :
    - robot : Structure contenant le modèle du robot.
    - θ : Configuration initiale des angles articulaires (7x1).
    - Pd : Position désirée de l'effecteur final (3x1).
    - dt : Pas de temps pour l'intégration (scalaire).
    - err_ts : Tolérance sur l'erreur de position (scalaire).
"""
function MCI(robot, θ, Pd, dt, err_ts)
    # Initialisation
    q = θ  # Configuration articulaire initiale
    Q_tot = []
    Q_tot = vcat(Q_tot, q)  # Initialisation avec q (vecteur 7x1, pas de crochet [])
    condition = false  # Critère d'arrêt
    while !condition
        # Étape 1 : Calcul de la cinématique directe
        X = MGD(q, robot)  # Matrice homogène 4x4 de l'effecteur
        Pe = X[1:3, 4]  # Position actuelle de l'effecteur (3x1)

        # Étape 2 : Calcul de l'erreur de position
        epsilon_p = Pd - Pe  # Erreur de position (3x1)

        # Étape 3 : Calcul de la Jacobienne
        J = Jacobian(q, robot, Pe)  # Jacobienne 6x7 (3 position + 3 orientation)
        J_translation = J[3, :]  # Extraction des 3 premières lignes pour la translation
        #print("size J_z = $(size(J_translation))")
        # Étape 4 : Résolution par pseudo-inverse
        
        q_dot = pinv(J_translation) * epsilon_p[3]  # Résolution pour les vitesses articulaires (7x1)

        # Étape 5 : Mise à jour des angles articulaires par intégration
        q_dot = q_dot'
        q = q + q_dot * dt
        
        # Sauvegarde dans Q_tot
        Q_tot = vcat(Q_tot, q)  # Concaténer verticalement, ajouter une nouvelle ligne

        # Étape 6 : Vérification des conditions d'arrêt
        if all(abs.(epsilon_p[3]) .< err_ts)
            condition = true
        end
    end

    return Q_tot
end

function MCI_CoM(robot, θnit, CoM_d, dt, err_CoM)
    # Initialisation
    q = θnit  # Configuration articulaire initiale
    Q_tot = []
    Q_tot = vcat(Q_tot, q)  # Initialisation avec q (vecteur 7x1, pas de crochet [])
    condition = false  # Critère d'arrêt
    while !condition
        # Étape 1 : Calcul de la cinématique directe
        CoM_Init = CoM(q, robot)  # Matrice homogène 4x4 de l'effecteur
        CoM_i = CoM_Init[1:3]  # Position actuelle de l'effecteur (3x1)

        # Étape 2 : Calcul de l'erreur de position
        epsilon_CoM = CoM_d[1:3] - CoM_i  # Erreur de position (3x1)

        # Étape 3 : Calcul de la Jacobienne
        J = JacobianCoM(q, robot, CoM_Init)  # Jacobienne 3x7 (3 position + 3 orientation)
        J_plan = J[1:2, :]

        # Étape 4 : Résolution par pseudo-inverse
        q_dot = pinv(J_plan) * epsilon_CoM[1:2]  # Résolution pour les vitesses articulaires (7x1)
        #q_dot = q_dot';

        # Étape 5 : Mise à jour des angles articulaires par intégration
        q = q + q_dot * dt

        # Sauvegarde dans Q_tot
        Q_tot = vcat(Q_tot, q)  # Concaténer verticalement, ajouter une nouvelle ligne

        # Étape 6 : Vérification des conditions d'arrêt
        if all(norm(epsilon_CoM[1:2]) < err_CoM)
            condition = true
        end
    end

    return Q_tot
end

function cmd_translation_z(robot, θinit, pD, dt, err_ts) # Tâche_1
    # Générer les configurations articulaires avec MCI
    Q = MCI(robot, θinit, pD, dt, err_ts)
    N = Int(length(Q) / 7)
    trajectoire = []

    # Initialiser une matrice pour enregistrer les trajectoires de toutes les articulations
    trajectoires_articulations = zeros(N, 7)  # N lignes, 7 colonnes pour chaque articulation

    for i = 1:N
        #Vecteur d'angle articulaire du robot
        s = (7 * i) - 6 #start
        e = (7 * (i + 1)) - 7 #end

        local T = MGD(Q[s:e], robot)
        push!(trajectoire, T[3, 4]) #Extraction de la position en z

        # Enregistrer les positions angulaires pour chaque articulation
        trajectoires_articulations[i, :] = Q[s:e]

        #Envoie de la commande a coppelia
        setjointposition(clientID, Q[s:e], 7, 0, objectname_kuka)
        sleep(dt)
    end
    trace_trajectoire(trajectoire)
    trace_trajectoires_articulations(trajectoires_articulations, "trajectoires_articulations_transition_z.png")
end


function cmd_plan(robot, θinit, CoM_d, dt, err_CoM) # Tâche_2

    # Générer les configurations articulaires avec MCI_CoM
    Q = MCI_CoM(robot, θinit, CoM_d, dt, err_CoM)
    N = Int(length(Q) / 7)
    trajectoire_CoM = []
    trajectoires_articulations = zeros(N, 7)  # N lignes, 7 colonnes pour chaque articulations

    for i = 1:N
        #Vecteur d'angle articulaire du robot
        s = (7 * i) - 6 #start
        e = (7 * (i + 1)) - 7 #end
        q_i = Q[s:e]  # Configuration articulaire actuelle

        # Calcul du centre de masse pour cette configuration
        CoM_i = CoM(q_i, robot)
        push!(trajectoire_CoM, CoM_i[1:3])  # Sauvegarde du centre de masse

        # Enregistrer les positions angulaires pour chaque articulation
        trajectoires_articulations[i, :] = q_i

        #Envoie de la commande a coppelia
        setjointposition(clientID, Q[s:e], 7, 0, objectname_kuka)
        sleep(dt)
    end
    # Tracé des trajectoires
    trace_trajectoire_com(trajectoire_CoM)
    trace_trajectoires_articulations(trajectoires_articulations, "trajectoires_articulations_CoM.png")
end


function calcul_hierarchie_2_taches(robot, θinit, pD, CoM_d, dt, err_ts, err_CoM) # Tâche_1 (CoM => + importante) et 2 (translation en z)
    q = θinit  # Configuration articulaire initiale
    Q_tot = []
    Q_tot = vcat(Q_tot, q)  # Initialisation avec q (vecteur 7x1, pas de crochet [])
    condition = false  # Critère d'arrêt
    
    while !condition
        # Calcul epsilon de t1
        CoM_Init = CoM(q, robot)  # Matrice homogène 4x4 de l'effecteur
        CoM_i = CoM_Init[1:3]  # Position actuelle de l'effecteur (3x1)
        e1 = CoM_d[1:3] - CoM_i  # Erreur de position (3x1)
        Ja = JacobianCoM(q, robot, CoM_Init)
        J1 = Ja[1:2, :]

        # Calcul epsilon de t2
        X = MGD(q, robot)  # Matrice homogène 4x4 de l'effecteur
        pE = X[1:3, 4]  # Position actuelle de l'effecteur (3x1)
        e2 = pD - pE  # Erreur de position (3x1)
        Jb = Jacobian(q, robot, pE)  # Jacobienne 6x7 (3 position + 3 orientation)
        J2 = Jb[3, :] 
        J2 = J2'

        #Calcul des tâches 
        P1 = I(7) - pinv(J1)*J1
        u1 = pinv(J1) * e1[1:2]
        u2 = pinv(J2*P1)*(e2[3] - J2*u1)

        #Intégration
        q_dot = u1 + u2 # Résolution pour les vitesses articulaires (7x1)
        q = q + q_dot * dt

        # Sauvegarde dans Q_tot
        Q_tot = vcat(Q_tot, q)

        if all((norm(e1[1:2]) .< err_CoM) .&& (abs.(e2[3]) .< err_ts))
            condition = true
        end
    end
    return Q_tot
end


function cmd_hierarchie_2_taches(robot, θinit, pD, CoM_d, dt, err_ts, err_CoM)
    # Générer les configurations articulaires avec MCI_CoM
    Q = calcul_hierarchie_2_taches(robot, θinit, pD, CoM_d, dt, err_ts, err_CoM) # Tâche_1 (CoM => + importante) et 2 (translation en z)
    N = Int(length(Q) / 7)
    trajectoire_parallele_z = []
    trajectoire_parallele_CoM = []
    trajectoires_articulations_parallele = zeros(N, 7)  # N lignes, 7 colonnes pour chaque articulations

    for i = 1:N
        #Vecteur d'angle articulaire du robot
        s = (7 * i) - 6 #start
        e = (7 * (i + 1)) - 7 #end
        q_i = Q[s:e]  # Configuration articulaire actuelle

        # Calcul du centre de masse pour cette configuration
        CoM_i = CoM(q_i, robot)
        push!(trajectoire_parallele_CoM, CoM_i[1:3])  # Sauvegarde du centre de masse

        T = MGD(q_i, robot)
        push!(trajectoire_parallele_z, T[3, 4]) #Extraction de la position en z

        trajectoires_articulations_parallele[i, :] = q_i

        #Envoie de la commande a coppelia
        setjointposition(clientID, Q[s:e], 7, 0, objectname_kuka)
        sleep(dt)
    end
    # Tracé des trajectoires
    trace_trajectoire_com(trajectoire_parallele_CoM)
    trace_trajectoire(trajectoire_parallele_z)
    trace_trajectoires_articulations(trajectoires_articulations_parallele_z, "trajectoires_articulations_parallele.png")
end


# Fonction de trace ----------------------------------------------------------------------------

function trace_trajectoire(trajectoire_z)
    # Nombre de pas
    nombre_de_pas = 1:length(trajectoire_z)

    # Tracé de la trajectoire
    p = plot(
        nombre_de_pas, trajectoire_z,
        title="Trajectoire en fonction du nombre de pas",
        xlabel="Nombre de pas",
        ylabel="Position en Z",
        legend=false,
        lw=2,
        marker=:circle,
        color=:blue
    )
    display(p)
    # Sauvegarder le graphique
    savefig("trajectoire_z.png")
end

function trace_trajectoire_com(trajectoire_com)
    # Extraction des coordonnées x, y, z
    trajectoire_x = [p[1] for p in trajectoire_com]
    trajectoire_y = [p[2] for p in trajectoire_com]
    trajectoire_z = [p[3] for p in trajectoire_com]

    # Nombre de pas
    nombre_de_pas = 1:size(trajectoire_com, 1)  # Nombre de lignes dans la matrice

    # Tracé des trajectoires 2D
    p1 = plot(trajectoire_x, trajectoire_y, color=:blue, title="Trajectoire CoM de y en fonction de x", xlabel="CoM x", ylabel="CoM y")
    display(p1)
    savefig("trajectoire_CoM.png")

    # Création du graphique 3D - Effectuez l'affichage après la fin de la simulation
    p2 = plot(trajectoire_x, trajectoire_y, trajectoire_z, lw=2, label="Trajectoire", xlabel="X", ylabel="Y", zlabel="Z", title="Trajectoire 3D")

    # Affichage du point final
    final_point_x = trajectoire_x[end]
    final_point_y = trajectoire_y[end]
    final_point_z = trajectoire_z[end]

    # Arrondir les coordonnées à 4 chiffres significatifs
    rounded_x = round(final_point_x, digits=4)
    rounded_y = round(final_point_y, digits=4)
    rounded_z = round(final_point_z, digits=4)

    # Ajouter une annotation pour afficher les coordonnées arrondies
    coords_text = "($rounded_x, $rounded_y, $rounded_z)"
    annotate!(final_point_x, final_point_y, final_point_z, text(coords_text, :red, 10))

    # Affichage et sauvegarde
    display(p2)
    savefig("trajectoire3D_com.png")
end



function trace_trajectoires_articulations(trajectoires_articulations, save_name)
    # Nombre de pas
    nombre_de_pas = 1:size(trajectoires_articulations, 1)  # Nombre de lignes dans la matrice

    # Tracé des trajectoires pour chaque articulation
    p1 = plot(title=save_name, xlabel="Nombre de pas", ylabel="Position angulaire (rad)")

    for j in 1:7
        plot!(
            nombre_de_pas, trajectoires_articulations[:, j],
            label="Articulation $j", lw=2
        )
    end
    display(p1)

    # Sauvegarder le graphique
    savefig(save_name)
end
