{ pkgs }:

pkgs.stdenv.mkDerivation {
  name = "can_pkg";

  src = ./gen_tool;
  
  # Some deps needed to generate the output DBC
  buildInputs = [
    pkgs.python311
    pkgs.python311Packages.cantools
  ];

  # Specify the output of the build process
  # In this case, it will be the generated file
  installPhase = ''
    mkdir -p $out
    python3 ./json_to_dbc.py ./can_descriptor.json ./dbc-output/car Orion RMS_PM Megasquirt
    mv ./dbc-output/car.dbc $out/car.dbc
  '';
}
