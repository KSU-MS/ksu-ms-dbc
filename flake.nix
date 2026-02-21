{
  description = "A very basic flake that generates the DBCs by running a python script";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
  };

  outputs =
    {
      flake-utils,
      self,
      nixpkgs,
    }:
    # Default for all nixpkgs
    let
      pkg_overlay = final: prev: {
        can_pkg = final.callPackage ./default.nix { };
      };
      custom_overlays = [ pkg_overlay ];
    in
    {
      overlays.default = custom_overlays;
    }

    # Add a package export for each system
    // flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ pkg_overlay ];
        };
      in
      {
        packages.default = pkgs.can_pkg;
      }
    );
}
