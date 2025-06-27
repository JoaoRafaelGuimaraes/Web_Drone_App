import numpy as np
from shapely.geometry import Point, Polygon
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Circle, Patch
from matplotlib import animation
from matplotlib.lines import Line2D

# --- Espécies e características ---
radii_map = {'Pequi': 1.5, 'Baru': 3.5, 'Capim': 0.2}
germ_rate_map = {'Pequi': 0.5, 'Baru': 0.6, 'Capim': 0.9}
color_map = {'Pequi': 'tab:red', 'Baru': 'tab:blue', 'Capim': 'tab:green'}

polygon = None
germ_rates = None
radii = None
N = None
P_OUT = None
colors = None
species_counts = None
poly_coords = None

OUT_PENALTY_FACTOR = 10.0

def repair_to_polygon(pt):
    global polygon
    while True:
        if polygon.contains(Point(pt)):
            return pt
        minx, miny, maxx, maxy = polygon.bounds
        pt = [np.random.uniform(minx, maxx), np.random.uniform(miny, maxy)]

def objetivo(positions, alpha):
    global germ_rates, N, polygon, radii, P_OUT
    gain = sum(germ_rates[i] for i in range(N) if polygon.contains(Point(positions[i])))

    
    dispersion = 0.0
    count = 0
    for i in range(N):
        for j in range(i+1, N):
            dispersion += np.linalg.norm(positions[i] - positions[j])
            count += 1
    if count > 0:
        dispersion /= count

    penalty = 0.0
    for i in range(N):
        for j in range(i+1, N):
            d = np.hypot(*(positions[i] - positions[j]))
            overlap = radii[i] + radii[j] - d
            if overlap > 0:
                penalty += overlap**2
    for p in positions:
        if not polygon.contains(Point(p)):
            penalty += P_OUT * OUT_PENALTY_FACTOR

    return gain + dispersion - alpha * penalty

def random_individual():
    global polygon, N
    pts = []
    minx, miny, maxx, maxy = polygon.bounds
    while len(pts) < N:
        x, y = np.random.uniform(minx, maxx), np.random.uniform(miny, maxy)
        if polygon.contains(Point(x, y)):
            pts.append([x, y])
    return np.array(pts)

def tournament_selection(pop, fits, k=3):
    idxs = np.random.choice(len(pop), k, replace=False)
    best = idxs[0]
    for idx in idxs[1:]:
        if fits[idx] > fits[best]:
            best = idx
    return pop[best]

def crossover(p1, p2, alpha_blx=0.3):
    global N
    child = np.zeros_like(p1)
    for i in range(N):
        for d in range(2):
            cmin, cmax = min(p1[i,d], p2[i,d]), max(p1[i,d], p2[i,d])
            I = cmax - cmin
            low, high = cmin - alpha_blx * I, cmax + alpha_blx * I
            child[i,d] = np.random.uniform(low, high)
        child[i] = repair_to_polygon(child[i])
    return child

def mutate(ind, sigma=0.9):
    global N
    child = ind + np.random.randn(N,2) * sigma
    for i in range(N):
        child[i] = repair_to_polygon(child[i])
    return child

# --- Parâmetros do GA ---
pop_size = 100
cx_rate = 0.8
mut_rate = 0.3
alpha_min, alpha_max = 1.0, 10.0

population = []
best_history = []
best_fitness_history = []
stop = False
aux = 0
ant_best = -np.inf
gen = 0

def evolution_loop(points, sp_count):
    global polygon, species_counts, radii, germ_rates, colors, N, P_OUT, poly_coords
    global population, best_history, best_fitness_history, stop, aux, ant_best, gen
    poly_coords = points
    # poly_coords = [(0.0, 0.0), (101.11664232027789, -1.6329974381833878), (-4.650380156441325, -143.6702567433517)]
    polygon = Polygon(poly_coords)
    species_counts = sp_count

    species_list = [sp for sp, count in species_counts.items() for _ in range(count)]
    N = len(species_list)
    radii = np.array([radii_map[sp] for sp in species_list])
    germ_rates = np.array([germ_rate_map[sp] for sp in species_list])
    colors = [color_map[sp] for sp in species_list]
    P_OUT = np.sum(germ_rates)

    population = [random_individual() for _ in range(pop_size)]
    best_history = []
    best_fitness_history = []
    stop = False
    aux = 0
    ant_best = -np.inf
    gen = 0

    while not stop and gen<=150:
        alpha = alpha_min + (alpha_max - alpha_min) * gen / 100
        fits = np.array([objetivo(ind, alpha) for ind in population])
        best_idx = np.argmax(fits)
        best_fit = fits[best_idx]
        avg_fit = np.mean(fits)
        print(f"[GA] Geração {gen+1} | α={alpha:.2f} | Melhor: {best_fit:.2f} | Média: {avg_fit:.2f} | Aux: {aux}")
        best_history.append(population[best_idx].copy())
        best_fitness_history.append(best_fit)

        new_pop = [population[best_idx].copy()]
        while len(new_pop) < pop_size:
            p1 = tournament_selection(population, fits, k=3)
            p2 = tournament_selection(population, fits, k=3)
            if np.random.rand() < cx_rate:
                c1 = crossover(p1, p2)
                c2 = crossover(p2, p1)
            else:
                c1, c2 = p1.copy(), p2.copy()
            if np.random.rand() < mut_rate:
                c1 = mutate(c1)
            if np.random.rand() < mut_rate:
                c2 = mutate(c2)
            new_pop.extend([c1, c2])
        population = new_pop[:pop_size]

        if best_fit <= ant_best and gen > 50:
            aux += 1
        else:
            aux = 0
        if aux > 20:
            stop = True
        ant_best = best_fit
        gen += 1
    animation_plot()

def animation_plot():
    global polygon, poly_coords, radii, colors, N, species_counts, best_history, best_fitness_history

    minx, miny, maxx, maxy = polygon.bounds
    width = maxx - minx
    height = maxy - miny
    aspect_ratio = width / height

    fig_height = 6
    fig_width = fig_height * aspect_ratio

    fig, ax = plt.subplots(figsize=(fig_width, fig_height))
    ax.set_xlim(minx, maxx)
    ax.set_ylim(miny, maxy)
    ax.set_aspect('equal')
    ax.axis('off')

    patch = plt.Polygon(poly_coords, closed=True, edgecolor='black', fill=False)
    ax.add_patch(patch)

    circles = [Circle((0, 0), radii[i], color=colors[i], alpha=0.7) for i in range(N)]
    for c in circles:
        ax.add_patch(c)

    present_species = set(species for species in species_counts if species in color_map)
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', label=specie,
               markerfacecolor=color_map[specie], markersize=8)
        for specie in present_species
    ]
    ax.legend(handles=legend_elements, loc='lower right', frameon=True, fontsize='small')

    def init():
        return circles

    def animate(i):
        pts = best_history[i]
        for idx, c in enumerate(circles):
            c.center = pts[idx]
        ax.set_title(f'Geração {i+1}, Fitness: {best_fitness_history[i]:.2f}')
        return circles

    anim = animation.FuncAnimation(
        fig, animate, init_func=init,
        frames=len(best_history), interval=200, blit=True
    )

    Writer = animation.writers['ffmpeg']
    writer = Writer(fps=5, metadata=dict(artist='GA'), bitrate=1800)
    anim.save(
        'ga_animation.mp4',
        writer=writer,
        dpi=200,
        savefig_kwargs={'pad_inches': 0}
    )

    print("Vídeo salvo como 'ga_animation.mp4'")
