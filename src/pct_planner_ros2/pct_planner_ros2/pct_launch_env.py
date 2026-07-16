import os


DEFAULT_VENV_SITE = os.environ.get(
    "PCT_VENV_SITE",
    "/home/orin/venv/m20_nav_cupy/lib/python3.10/site-packages",
)

CUDA_LIBRARY_PACKAGES = (
    "cublas",
    "cuda_cupti",
    "cuda_nvrtc",
    "cuda_runtime",
    "cudnn",
    "cufft",
    "curand",
    "cusolver",
    "cusparse",
    "nccl",
    "nvjitlink",
    "nvtx",
)


def cuda_library_path_substitutions(venv_site):
    paths = []
    for package_name in CUDA_LIBRARY_PACKAGES:
        paths.extend([venv_site, f"/nvidia/{package_name}/lib:"])
    return paths
