#! /bin/bash
PATH_PACKAGE=$(dirname "$(pwd)")
#echo $PATH_PACKAGE

# Move experiment data to the other folder to plot data
cp $PATH_PACKAGE/../experiment_data/*.txt $PATH_PACKAGE/../experiment_data_check/

# Remove every data for the next experiment
rm $PATH_PACKAGE/../experiment_data/*.txt

exit 0
