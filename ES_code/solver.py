import math as m
import copy
import ray
import numpy as np

from glo import *
np.random.seed(NUMPY_SEED)

class Parameter():
    def __init__(self, initial_value, noise_mu=0, noise_sigma=1, lb=None, ub=None):
        assert(len(initial_value.shape)==2) # 2d array
        # Basic
        self.value = initial_value
        # Noise
        self.noise_sigma = noise_sigma
        self.noise_mu = noise_mu
        # Clipping
        self.lb = lb
        self.ub = ub

class ES():
    # General ES solver, override your own evaluator which depends on the application
    def __init__(self, params, npop, sigma, alpha):
        self.params = params
        self.npop = npop
        self.sigma = sigma
        self.alpha = alpha
    
    def get_perturbed_models(self):
        # npop different perturbations to each parameter
        perturbations = []
        for p in self.params:
            perturbations.append((np.random.randn(self.npop, p.value.shape[0], p.value.shape[1]) * p.noise_sigma) + p.noise_mu)

        perturbed_models = []
        for j in range(self.npop):
            perturbed_params_set = [copy.deepcopy(p) for p in self.params]
            for idx, p in enumerate(perturbed_params_set): 
                perturbed_params_set[idx].value += self.sigma * perturbations[idx][j]

                # If implemented own clipping method by overriding
                try: perturbed_params_set[idx].value = self.clip(perturbed_params_set[idx]) 
                except NotImplementedError: pass

            perturbed_models.append(perturbed_params_set)

        return perturbed_models, perturbations
    
    def clip(self, params):
        raise NotImplementedError

    # Implement your own, should return a list of scores, one per perturbed model
    def evaluate_perturbed_models(self):
        raise NotImplementedError()
    
    def recombine_new_base_model(self, scores, perturbations):    
        # New model is a weighted combination (based on resulting rewards) of perturbed models + old model
        eps = 1e-5
        A = (scores - np.mean(scores)) / (np.std(scores)+eps)
        new_params = []
        
        for idx, p in enumerate(self.params):
            p.value += self.alpha/(self.npop*self.sigma) * np.dot(perturbations[idx].transpose(1, 2, 0), A)
            try: p.value = self.clip(p) 
            except NotImplementedError: pass
        return

    def run_one_iteration(self, workers):
        # npop different perturbed models
        perturbed_models, perturbations_lst = self.get_perturbed_models()
        # npop different scores 
        scores = self.evaluate_perturbed_models(perturbed_models, workers)
        # Use scores to weigh the perturbed versions of each param into its new value
        self.recombine_new_base_model(scores, perturbations_lst)
        return self.params

class ES_parallel(ES):
    # Specialized ES solver which evaluates models in a parallel manner using ray workers
    def __init__(self, params, npop, sigma, alpha, num_workers):
        super().__init__(params, npop, sigma, alpha)
        self.num_workers = num_workers
    
    def evaluate_perturbed_models(self, perturbed_models, workers):
        scores = np.zeros(len(perturbed_models))
        
        # Fully saturate all the workers to start
        batches_for_workers = self.group_jobs_for_workers(models=perturbed_models, num_workers=self.num_workers)
        ongoing_ids = [workers[j].evaluate_models.remote(batches_for_workers[j]) for j in range(len(workers))]
        returns = ray.get(ongoing_ids)

        for r in returns:
            for model_data in r:
                model_idx, score_before, score_after = model_data
                # Evaluate on sum of scores: before and after shaking
                scores[model_idx] = score_before + score_after
        return scores

    def group_jobs_for_workers(self, models, num_workers):
        assert num_workers <= len(models), f"Increase population size by {num_workers-len(models)} as some workers are idle"

        batch_size = m.floor(len(models)/num_workers)
        tail = len(models) - batch_size*num_workers

        batches = []
        idx = 0

        for j in range(tail):
            batch = []
            for i in range(batch_size+1):
                batch.append([idx, models.pop(0)])
                idx += 1

            batches.append(batch)
            
        while len(models) > 0:
            batch = []
            for i in range(batch_size):
                batch.append([idx, models.pop(0)])
                idx += 1

            batches.append(batch)

        return batches


    def clip(self, perturbed_params):
        val = np.copy(perturbed_params.value)
        for i in range(val.shape[1]):
            if not (perturbed_params.lb[i] is None and perturbed_params.ub[i] is None):
                val[:,i] = np.clip(val[:,i], perturbed_params.lb[i], perturbed_params.ub[i])
        return val