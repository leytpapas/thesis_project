# $(dirname $(realpath $0))
echo "Executing in $(dirname $(realpath $0))/setup.py "
python3 "$(dirname $(realpath $0))/cython_setup.py" build_ext --inplace