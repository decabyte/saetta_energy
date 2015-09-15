# saetta_energy/RegressionResult.msg
#   message for reporting the regression results

Header header

string type             # type of regression
float64 mse             # mean squared error
float64[] coeff         # regression coefficients

float64[] int_pred      # prediction intervals
float64[] int_conf      # confidence intervals
