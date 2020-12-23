Change Log
==========

2.0
----------------------
- 2020-12-22: updated external functions with newer version of contact model. This might cause tiny differences, although evaluation of the nominal functions gives similar results (up to 1e-8).

- 2020-07-20: corrected bug in metabolic energy models (Umberger2003, Umberger2010, Uchida2016); 10*lmopt had been used as normalized maximal muscle fiber contraction velocity rather than 10. Thanks Tim van der Zee for reporting. This suggests that the results obtained in the paper when comparing the different metabolic energy models might be incorrect.

- 2020-04-06: code for writing .txt file with cpp code for building OpenSim model programmatically. 

- 2020-03-31: PredSim_all_v2 that contains numerous changes wrt PredSim_all, including adjusted collocation scheme and support for parallel computing. More details at the top of the script.

- 2020-03-25: code for estimation of polynomial coefficients adjusted to be less subject-specific.    

- 2020-03-25: added code to extract and save MT-parameters from a given model.           
