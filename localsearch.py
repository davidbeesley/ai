from queue import PriorityQueue
from threading import Timer


################################################################################################
# localsearch.py
# David Beesley
#
# Implementation of common localsearch algorithms.
#
# Todo: Simulated Annealing and Genetic Algs, Random Restart
################################################################################################


class Log:
    f_open = False
    f = None

    @staticmethod
    def write(*args, **kargs):
        if not Log.f_open:
            Log.f = open("localsearch.log", 'w')
            Log.f_open=True;
        print(*args, file=Log.f, **kargs)




class State:
    def get_neighbors(self):
        raise NotImplementedError("Should have implemented this")

    # True or False
    def is_objective(self):
        raise NotImplementedError("Should have implemented this")

    # Value to be maximized
    def score(self):
        raise NotImplementedError("Should have implemented this")

    ################################################################################################
    # Algorithm specific functions.
    ################################################################################################
    def breed(self, other):
        raise NotImplementedError("Should have implemented this for genetic algorithm")

    def randomize(self):
        raise NotImplementedError("Should have implemented this for random restart")


    def __lt__(self, other):
        return self.score() < other.score()


class LocalSearchAlgorithm:
    def __init__(self, initial_state):
        self.initial_state = initial_state
        self.interrupted = False
        self.finished = False

    # returns solution state, True if objective
    def run(self, timeout=10):
        Log.write("Starting")
        t = Timer(timeout, self.interrupt)
        t.start()
        s = self.solve()
        self.finished = True
        return s

    def solve(self):
        raise NotImplementedError("Should have implemented this")

    def interrupt(self):
        if not self.finished:
            Log.write("timed out")
        self.interrupted = True


class GreedyHillClimb(LocalSearchAlgorithm):
    def solve(self):
        state = self.initial_state
        while not state.is_objective() and not self.interrupted:
            current_score = state.score()
            neighbors = sorted(state.get_neighbors(), reverse=True)
            if len(neighbors) is 0:
                return state, False
            if neighbors[0].score() <= current_score:
                return state, False
            state = neighbors[0]

        return state, state.is_objective()


class BeamSearch(LocalSearchAlgorithm):
    def __init__(self, initial_state, beam_size=100):
        super().__init__(initial_state)
        self.beam_size = beam_size

    def solve(self):
        best_score = self.initial_state.score()
        bssf = self.initial_state
        states = [self.initial_state]

        while len(states) > 0 and not self.interrupted:
            new_states = []
            for state in states:
                if state.is_objective():
                    return state, True
                if state.score() > best_score:
                    best_score = state.score()
                    bssf = state
                new_states += state.get_neighbors()
            states = sorted(new_states, reverse=True)
            if len(states) > self.beam_size:
                states = states[:self.beam_size]
        return bssf, False


class GreedyBeamSearch(LocalSearchAlgorithm):
    def __init__(self, initial_state, beam_size=100):
        super().__init__(initial_state)
        self.beam_size = beam_size

    def solve(self):
        best_score = self.initial_state.score()
        bssf = self.initial_state
        states = PriorityQueue(maxsize=2 * self.beam_size)
        states.put(self.initial_state)

        while not states.empty() and not self.interrupted:
            state = states.get()
            if state.is_objective():
                return state, True
            Log.write("score:", state.score())
            if state.score() > best_score:
                best_score = state.score()
                bssf = state
            for s in state.get_neighbors():
                if states.full():
                    states = self.resize_queue(states)
                states.put(s)
        Log.write("Ending qsize", states.qsize())
        return bssf, False

    def resize_queue(self, full_queue):
        Log.write("Queue resized")
        queue = PriorityQueue()
        for i in range(self.beam_size):
            queue.put(full_queue.get())
        return queue
