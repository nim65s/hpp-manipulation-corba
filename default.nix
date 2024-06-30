{
  lib,
  cmake,
  hpp-corbaserver,
  hpp-manipulation-urdf,
  pkg-config,
  python3Packages,
}:

python3Packages.buildPythonPackage {
  pname = "hpp-manipulation-corba";
  version = "5.0.0";
  pyproject = false;

  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.unions [
      ./CMakeLists.txt
      ./doc
      ./idl
      ./include
      ./package.xml
      ./src
      ./tests
    ];
  };

  strictDeps = true;

  nativeBuildInputs = [
    cmake
    pkg-config
  ];
  propagatedBuildInputs = [
    hpp-corbaserver
    hpp-manipulation-urdf
    python3Packages.omniorbpy
  ];

  enableParallelBuilding = false;

  doCheck = true;

  meta = {
    description = "Corba server for manipulation planning";
    homepage = "https://github.com/humanoid-path-planner/hpp-manipulation-corba";
    license = lib.licenses.bsd2;
    maintainers = [ lib.maintainers.nim65s ];
  };
}
