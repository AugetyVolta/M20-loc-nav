DEFAULT_VENV_SITE = (
    "/home/ubuntu/xlab/M20-loc-nav/"
    ".venv/m20_nav_jazzy/lib/python3.12/site-packages"
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
