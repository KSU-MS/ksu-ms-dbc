{ pkgs }:

pkgs.stdenv.mkDerivation {
  name = "can_pkg";

  src = ./.;

  # Some deps needed to generate the output DBC
  buildInputs = [
    pkgs.python313Packages.cantools
  ];

  # Specify the output of the build process
  # In this case, it will be the generated file
  # car.dbc: the default vehicle (vehicles.json), what the firmware pins; every vehicle's
  # own in vehicles/ (kept out of the top level, which CI's code generators glob)
  installPhase = ''
    mkdir -p $out/vehicles
    python3 ./vehicles.py --out ./dbc-output
    mv ./dbc-output/car.dbc $out/car.dbc
    mv ./dbc-output/vehicles/*.dbc $out/vehicles/
  '';
}
