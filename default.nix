{ pkgs, python311Packages, fetchPypi }:

pkgs.stdenv.mkDerivation {
  name = "dbc_pkg";

  src = ./.;

  buildInputs = [
    (python311Packages.cantools.overridePythonAttrs (old: {
      src = fetchPypi {
        pname = "cantools";
        version = "39.4.4";
        hash = "sha256-bo6Ri2ZxpiqfOZBUbs5WI+Hetx3vsc74WplVrDAdqZ4=";
      };
      doCheck = false;
    }))
  ]; # Python as a build dependency

  # Define the build phase to execute the scripts
  buildPhase = ''
    python json_to_dbc.py can_descriptor.json ./dbc-output/ksu_can.dbc Orion RMS_PM Megasquirt
  '';

  # Specify the output of the build process
  # In this case, it will be the generated file
  installPhase = ''
    mkdir -p $out
    mv ./dbc-output/ksu_can.dbc $out/ksu_can.dbc
  '';
}
