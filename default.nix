{ pkgs }:

pkgs.stdenv.mkDerivation rec {
  name = "can_pkg";
  
  # Some deps needed to generate the output DBC
  buildInputs = [
    pkgs.python312
    pkgs.python312Packages.cantools
  ];

  # Specify the output of the build process
  # In this case, it will be the generated file
  installPhase = ''
    mkdir -p $out
    python3 ./json_to_dbc.py ./can_descriptor.json $out/car Orion RMS_PM Megasquirt
  '';
}
