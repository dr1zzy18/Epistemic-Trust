#!/usr/bin/env python

from epistemic.srv import choice, choiceResponse
import rospy
from pgmpy.models import BayesianNetwork
from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination


# Define the network structure
model = BayesianNetwork([('Age', 'Choice'), ('Test', 'Choice'), ('Information', 'Choice')])

# Define the CPT for Age
cpd_age = TabularCPD(variable='Age', variable_card=2, values=[[0.5], [0.5]],
                     state_names={'Age': ['3', '4']})

# Define the CPT for Test
cpd_test = TabularCPD(variable='Test', variable_card=2, values=[[0.5], [0.5]],
                      state_names={'Test': ['Pretest', 'Posttest']})

# Define the CPT for Information
cpd_information = TabularCPD(variable='Information', variable_card=2, values=[[0.5], [0.5]],
                             state_names={'Information': ['Accurate', 'Inaccurate']})

# Define the CPT for Choice given Age, Test, and Information
cpd_choice = TabularCPD(variable='Choice', variable_card=2,
                        values=[[0.7, 0.65, 0.7, 0.65, 0.7, 0.65, 0.9, 0.5],
                                [0.3, 0.35, 0.3, 0.35, 0.3, 0.35, 0.1, 0.5]],
                        evidence=['Age', 'Test', 'Information'],
                        evidence_card=[2, 2, 2],
                        state_names={'Choice': ['Familiar', 'Unfamiliar'],
                                     'Age': ['3', '4'],
                                     'Test': ['Pretest', 'Posttest'],
                                     'Information': ['Accurate', 'Inaccurate']})

# Add the CPTs to the model
model.add_cpds(cpd_age, cpd_test, cpd_information, cpd_choice)

# Verify the model
assert model.check_model()


def handle_choice(input):
    # Query the model with the evidence given in the input
    evidence = {}
    evidence['Age'] = str(input.age)
    evidence['Test'] = str(input.test)
    evidence['Information'] = str(input.accuracy)
    probability_choice = 0.000

    # Create an instance of the VariableElimination class and pass the Bayesian Network model as a parameter
    infer = VariableElimination(model)
    # Query the model with the given evidence and compute the posterior probability distribution over the variable 'M'
    q = infer.query(variables=['Choice'], evidence=evidence)
    if (str(input.type) == "Familiar"):
        probability_choice = q.values[0]
    else:
        probability_choice = q.values[1]
    # Return the response
    return choiceResponse(p_choice=probability_choice)




if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('choicePrediction')
    rospy.Service('choice_service', choice, handle_choice)
    rospy.spin()
