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
  installPhase = ''
    mkdir -p $out
    python3 ./json_to_dbc.py ./can_descriptor.json ./dbc-output/car Orion 20240625-Gen5-CAN-DB elcon Megasquirt messageNEW
    mv ./dbc-output/car.dbc $out/car.dbc
  '';
}
