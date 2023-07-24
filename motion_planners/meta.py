import time

from .lattice import lattice
from .lazy_prm import lazy_prm, lazy_prm_star, create_param_sequence
from .prm import prm
from .rrt import rrt
from .rrt_connect import rrt_connect, birrt
from .rrt_star import rrt_star
from .utils import INF

from .smoothing import smooth_path
from .utils import RRT_RESTARTS, RRT_SMOOTHING, INF, irange, elapsed_time, compute_path_cost, default_selector


def direct_path(start, goal, extend_fn, collision_fn):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    # TODO: version which checks whether the segment is valid
    if collision_fn(start) or collision_fn(goal):
        # TODO: return False
        return None
    path = list(extend_fn(start, goal))
    path = [start] + path
    if any(collision_fn(q) for q in default_selector(path)):
        return None
    return path
    # path = [start]
    # for q in extend_fn(start, goal):
    #     if collision_fn(q):
    #         return None
    #     path.append(q)
    # return path

def check_direct(start, goal, extend_fn, collision_fn):
    if any(collision_fn(q) for q in [start, goal]):
        print('start or goal in collision')
        return False
    return direct_path(start, goal, extend_fn, collision_fn)

#################################################################

def random_restarts(solve_fn, start, goal, distance_fn, sample_fn, extend_fn, collision_fn, path_cost_fn=None,
                    restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING,
                    success_cost=0., max_time=INF, max_solutions=1, max_cost=INF, verbose=False, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_time: Maximum runtime - float
    :param kwargs: Keyword arguments
    :return: Paths [[q', ..., q"], [[q'', ..., q""]]
    """
    start_time = time.time()
    paths = []
    path_costs = []
    path = check_direct(start, goal, extend_fn, collision_fn)
    if path is False:
        print('No direct path connecting start and goal')
        return None
    if path is not None and path_cost_fn(path) < max_cost:
        paths.append(path)

    while True:
        if (len(paths) >= max_solutions) or (elapsed_time(start_time) >= max_time):
            break
        attempt_time = (max_time - elapsed_time(start_time))
        path = solve_fn(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                        max_time=attempt_time, verbose=False, **kwargs)
        if path is None:
            continue
        path = smooth_path(path, extend_fn, collision_fn, max_iterations=smooth,
                           max_time=max_time-elapsed_time(start_time), verbose=False)
        path_cost = path_cost_fn(path)
        # print('Path cost:', path_cost)
        if path_cost > max_cost:
            continue
        paths.append(path)
        path_costs.append(path_cost)
        # if compute_path_cost(path, distance_fn) < success_cost:
        #     break
        # print(len(paths), 'paths', elapsed_time(start_time))
    # paths = sorted(paths, key=lambda path: path_cost_fn(path))
    paths = [path for _, path in sorted(zip(path_costs, paths))]
    if verbose:
        print('Solutions ({}): {} | Time: {:.3f}'.format(len(paths),
                                                         [(len(path), round(cost, 3))
                                                          for path, cost in zip(paths, path_costs)],
                                                         elapsed_time(start_time)))
    return paths

def solve_and_smooth(solve_fn, q1, q2, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs):
    return random_restarts(solve_fn, q1, q2, distance_fn, sample_fn, extend_fn, collision_fn,
                           restarts=0, max_solutions=1, **kwargs)

#################################################################

def solve(start, goal, distance_fn, sample_fn, extend_fn, collision_fn, algorithm='birrt',
          max_time=INF, max_iterations=INF, num_samples=100, smooth=None, smooth_time=INF, # TODO: smooth_iterations
          weights=None, circular={},
          cost_fn=None, success_cost=INF, verbose=False, **kwargs):
    # TODO: better shared default options
    # TODO: allow distance_fn to be skipped
    # TODO: return lambda function
    start_time = time.time()
    path = check_direct(start, goal, extend_fn, collision_fn)
    if (path is not None) or (algorithm == 'direct'):
        return path

    remaining_time = max_time - elapsed_time(start_time)
    if algorithm == 'prm': # TODO: cost_fn
        path = prm(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                   num_samples=num_samples)
    elif algorithm == 'lazy_prm':
        if weights is not None:
            distance_fn = None
        path = lazy_prm(start, goal, sample_fn, extend_fn, collision_fn, num_samples=num_samples,
                        max_time=remaining_time, weights=weights, circular=circular, distance_fn=distance_fn,
                        cost_fn=cost_fn, success_cost=success_cost, verbose=verbose)[0]
    elif algorithm == 'lazy_prm_star':
        if weights is not None:
            distance_fn = None
        param_sequence = create_param_sequence(initial_samples=num_samples)
        path = lazy_prm_star(start, goal, sample_fn, extend_fn, collision_fn, param_sequence=param_sequence,
                             max_time=remaining_time, weights=weights, circular=circular, distance_fn=distance_fn,
                             cost_fn=cost_fn, success_cost=success_cost, verbose=verbose) # **kwargs)
    elif algorithm == 'rrt':
        path = rrt(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                   max_iterations=max_iterations, max_time=remaining_time)
    elif algorithm == 'rrt_connect':
        path = rrt_connect(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                           max_iterations=max_iterations, max_time=remaining_time)
    elif algorithm == 'birrt':
        # TODO: checks the straight-line twice
        path = birrt(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                     max_iterations=max_iterations, max_time=remaining_time, smooth=None, **kwargs) # restarts=2
    elif algorithm == 'rrt_star':
        path = rrt_star(start, goal, distance_fn, sample_fn, extend_fn, collision_fn, radius=1,
                        max_iterations=max_iterations, max_time=remaining_time)
    elif algorithm == 'lattice':
        path = lattice(start, goal, extend_fn, collision_fn, distance_fn=distance_fn, max_time=INF)
    else:
        raise NotImplementedError(algorithm)

    remaining_time = min(smooth_time, max_time - elapsed_time(start_time))
    return smooth_path(path, extend_fn, collision_fn, # sample_fn=sample_fn,
                       distance_fn=distance_fn, cost_fn=cost_fn,
                       max_iterations=smooth, max_time=remaining_time, verbose=verbose)
