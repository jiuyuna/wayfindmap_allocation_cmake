#pragma once
#include "Individual.h"

class Population
{
public:
	Individual* ind[maxpop];
	int popsize;
	int indsize;
	Population(int pop_size, int ind_size);
	~Population();

	void initialize_population();
	void evaluate_population();


private:

};


