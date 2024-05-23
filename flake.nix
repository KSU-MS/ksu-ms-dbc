{
  description = "A flake for KSU-MS's DBC for our DAQ repo";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-23.11";
  };

  outputs = { self, nixpkgs }:
  let 
    dbc_overlay = final: prev: {
      can_pkg = final.callPackage ./default.nix { };
    };

    # Abstract what platform we are building for
    forAllSystems = function:
      nixpkgs.lib.genAttrs [
        "x86_64-linux"
        "aarch64-linux"
      ] (system: function nixpkgs.legacyPackages.${system});

  in {
    overlays.default = dbc_overlay;

    packages = forAllSystems (pkgs: {
      default = pkgs.callPackage ./default.nix {};
    });
  };
}
