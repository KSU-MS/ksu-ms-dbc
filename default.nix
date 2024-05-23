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
    mkdir -p $out
    python json_to_dbc.py can_descriptor.json $out/car Orion RMS_PM Megasquirt
  '';
}
