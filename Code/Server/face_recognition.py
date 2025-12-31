"""
Server-side facial recognition for autonomous operation.
Detects and recognizes faces in real-time from camera feed.
"""

import os
import cv2
import numpy as np
from typing import Optional, List, Tuple, Dict
from datetime import datetime
import json
from pathlib import Path

from logger import get_logger

logger = get_logger()


class FaceRecognition:
    """
    Real-time face detection and recognition using OpenCV.
    Supports training with labeled faces and persistent storage.
    """

    def __init__(self, face_data_dir: str = "face_data"):
        """
        Initialize face recognition system.

        Args:
            face_data_dir: Directory for storing face data and models
        """
        self.face_data_dir = Path(face_data_dir)
        self.face_data_dir.mkdir(exist_ok=True)

        # Paths for model and data
        self.model_path = self.face_data_dir / "face_model.yml"
        self.database_path = self.face_data_dir / "face_database.json"
        self.images_dir = self.face_data_dir / "images"
        self.images_dir.mkdir(exist_ok=True)

        # Initialize OpenCV face recognizer and detector
        try:
            self.recognizer = cv2.face.LBPHFaceRecognizer_create()

            # Load existing model if available
            if self.model_path.exists():
                self.recognizer.read(str(self.model_path))
                logger.info(f"Loaded face recognition model from {self.model_path}")
            else:
                logger.info("No existing face model found, starting fresh")

        except AttributeError:
            logger.error("cv2.face module not available. Install opencv-contrib-python")
            raise

        # Haar cascade for face detection
        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        self.detector = cv2.CascadeClassifier(cascade_path)

        if self.detector.empty():
            logger.error(f"Failed to load Haar cascade from {cascade_path}")
            raise ValueError("Haar cascade not loaded")

        # Load face database
        self.face_database = self._load_database()

        # Recognition parameters
        self.confidence_threshold = 100.0  # Lower is better, >100 = unknown
        self.min_face_size = (30, 30)  # Minimum face size to detect
        self.scale_factor = 1.2  # Detection scale factor
        self.min_neighbors = 5  # Detection quality parameter

        logger.info("Face recognition system initialized")

    def _load_database(self) -> Dict:
        """Load face database from JSON file."""
        if self.database_path.exists():
            with open(self.database_path, 'r') as f:
                db = json.load(f)
                logger.info(f"Loaded {len(db.get('faces', {}))} known faces from database")
                return db
        else:
            logger.info("Creating new face database")
            return {
                "faces": {},  # id -> {name, added_date, last_seen, sample_count}
                "next_id": 1
            }

    def _save_database(self) -> None:
        """Save face database to JSON file."""
        with open(self.database_path, 'w') as f:
            json.dump(self.face_database, f, indent=2)
        logger.debug("Face database saved")

    def detect_faces(self, frame: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """
        Detect faces in a frame.

        Args:
            frame: BGR image from camera

        Returns:
            List of (x, y, w, h) bounding boxes
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.detector.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=self.min_face_size
        )
        return faces

    def recognize_face(
        self,
        frame: np.ndarray,
        face_rect: Tuple[int, int, int, int]
    ) -> Tuple[Optional[int], float, Optional[str]]:
        """
        Recognize a detected face.

        Args:
            frame: BGR image from camera
            face_rect: (x, y, w, h) bounding box

        Returns:
            (face_id, confidence, name) or (None, confidence, None) if unknown
        """
        x, y, w, h = face_rect
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        face_roi = gray[y:y+h, x:x+w]

        try:
            face_id, confidence = self.recognizer.predict(face_roi)

            if confidence > self.confidence_threshold:
                # Unknown face
                return None, confidence, None
            else:
                # Known face
                face_info = self.face_database["faces"].get(str(face_id))
                if face_info:
                    name = face_info["name"]
                    # Update last seen timestamp
                    face_info["last_seen"] = datetime.now().isoformat()
                    self._save_database()
                    return face_id, confidence, name
                else:
                    logger.warning(f"Face ID {face_id} recognized but not in database")
                    return None, confidence, None

        except cv2.error as e:
            logger.error(f"Face recognition error: {e}")
            return None, float('inf'), None

    def detect_and_recognize(
        self,
        frame: np.ndarray,
        draw_boxes: bool = False
    ) -> List[Dict]:
        """
        Detect and recognize all faces in a frame.

        Args:
            frame: BGR image from camera
            draw_boxes: If True, draw bounding boxes and labels on frame

        Returns:
            List of face dictionaries with keys:
                - rect: (x, y, w, h)
                - id: face ID or None
                - name: person name or "Unknown"
                - confidence: recognition confidence
        """
        faces = self.detect_faces(frame)
        results = []

        for face_rect in faces:
            x, y, w, h = face_rect
            face_id, confidence, name = self.recognize_face(frame, face_rect)

            result = {
                "rect": (x, y, w, h),
                "id": face_id,
                "name": name if name else "Unknown",
                "confidence": confidence,
                "is_known": name is not None
            }
            results.append(result)

            if draw_boxes:
                # Draw rectangle
                color = (0, 255, 0) if name else (0, 0, 255)
                cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)

                # Draw label
                label = f"{name}" if name else "Unknown"
                if confidence < float('inf'):
                    label += f" ({confidence:.1f})"
                cv2.putText(
                    frame, label, (x+5, y+h+25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
                )

        return results

    def add_face_sample(
        self,
        frame: np.ndarray,
        name: str,
        face_id: Optional[int] = None
    ) -> Optional[int]:
        """
        Add a face sample for training.

        Args:
            frame: BGR image containing the face
            name: Person's name
            face_id: Existing ID to add sample to, or None for new person

        Returns:
            Face ID or None if no face detected
        """
        faces = self.detect_faces(frame)

        if len(faces) == 0:
            logger.warning("No face detected in sample image")
            return None

        if len(faces) > 1:
            logger.warning(f"Multiple faces detected ({len(faces)}), using largest")

        # Use largest face
        face_rect = max(faces, key=lambda r: r[2] * r[3])
        x, y, w, h = face_rect

        # Assign ID
        if face_id is None:
            face_id = self.face_database["next_id"]
            self.face_database["next_id"] += 1

            # Create database entry
            self.face_database["faces"][str(face_id)] = {
                "name": name,
                "added_date": datetime.now().isoformat(),
                "last_seen": datetime.now().isoformat(),
                "sample_count": 0
            }

        # Save face image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        face_roi = gray[y:y+h, x:x+w]

        sample_count = self.face_database["faces"][str(face_id)]["sample_count"]
        image_path = self.images_dir / f"{face_id}.{sample_count}.jpg"
        cv2.imwrite(str(image_path), face_roi)

        # Update sample count
        self.face_database["faces"][str(face_id)]["sample_count"] += 1
        self._save_database()

        logger.info(f"Added face sample for '{name}' (ID: {face_id}, sample {sample_count})")
        return face_id

    def train_recognizer(self) -> bool:
        """
        Train the face recognizer with all stored samples.

        Returns:
            True if training succeeded, False otherwise
        """
        logger.info("Training face recognizer...")

        face_samples = []
        labels = []

        # Load all face images
        for image_file in self.images_dir.glob("*.jpg"):
            try:
                # Parse filename: {id}.{sample_num}.jpg
                face_id = int(image_file.stem.split('.')[0])

                # Load and process image
                img = cv2.imread(str(image_file), cv2.IMREAD_GRAYSCALE)
                if img is not None:
                    face_samples.append(img)
                    labels.append(face_id)

            except (ValueError, IndexError) as e:
                logger.warning(f"Skipping invalid image file: {image_file}")

        if len(face_samples) == 0:
            logger.warning("No face samples found for training")
            return False

        # Train recognizer
        try:
            self.recognizer.train(face_samples, np.array(labels))
            self.recognizer.write(str(self.model_path))
            logger.info(f"Training complete: {len(face_samples)} samples, "
                       f"{len(np.unique(labels))} unique faces")
            return True

        except cv2.error as e:
            logger.error(f"Training failed: {e}")
            return False

    def get_known_faces(self) -> Dict[int, Dict]:
        """
        Get list of all known faces.

        Returns:
            Dictionary mapping face ID to info dict
        """
        return {
            int(face_id): info
            for face_id, info in self.face_database["faces"].items()
        }

    def delete_face(self, face_id: int) -> bool:
        """
        Delete a face from the database.

        Args:
            face_id: Face ID to delete

        Returns:
            True if deleted, False if not found
        """
        face_id_str = str(face_id)

        if face_id_str not in self.face_database["faces"]:
            logger.warning(f"Face ID {face_id} not found in database")
            return False

        # Delete images
        for image_file in self.images_dir.glob(f"{face_id}.*.jpg"):
            image_file.unlink()

        # Remove from database
        del self.face_database["faces"][face_id_str]
        self._save_database()

        logger.info(f"Deleted face ID {face_id}")

        # Retrain if model exists
        if self.model_path.exists():
            self.train_recognizer()

        return True


if __name__ == '__main__':
    """Test face recognition system"""
    from logger import setup_logger
    import time

    setup_logger(log_level='INFO')

    # Initialize face recognition
    face_rec = FaceRecognition()

    # Try to open camera
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            logger.error("Cannot open camera")
            exit(1)

        logger.info("Camera opened. Press 'q' to quit, 's' to save sample, 't' to train")

        while True:
            ret, frame = cap.read()
            if not ret:
                logger.error("Failed to grab frame")
                break

            # Detect and recognize faces
            results = face_rec.detect_and_recognize(frame, draw_boxes=True)

            # Display info
            for result in results:
                logger.info(f"Face: {result['name']} (confidence: {result['confidence']:.1f})")

            # Show frame
            cv2.imshow('Face Recognition Test', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                name = input("Enter name for this face: ")
                face_id = face_rec.add_face_sample(frame, name)
                if face_id:
                    logger.info(f"Sample saved for {name} (ID: {face_id})")
            elif key == ord('t'):
                face_rec.train_recognizer()

        cap.release()
        cv2.destroyAllWindows()

    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
