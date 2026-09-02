source ~/miniconda3/etc/profile.d/conda.sh
conda activate xrobot312

conda install -y \
  -c robostack-jazzy \
  -c conda-forge \
  ros-jazzy-kdl-parser

export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"

python -c "import arx5_interface; print('ARX5 SDK OK')" || \
  python -m pip install --proxy http://127.0.0.1:7890 arx5-interface==0.1.3
